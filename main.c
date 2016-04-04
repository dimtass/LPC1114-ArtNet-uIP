/*-------------------------------------------------------------------------------------
 * Project: GiDT DMX-512-to-Ethernet controller
 * Purpose: Drives DMX-512 controllers via ethernet
 * Note(s):
 *      Usefull/needed files are common.h main.c connection.h connection.c 
 *      dmx.h dmx.c dm9k_uip.h dm9k_uip.c uip-conf.h clock-arch.c
 *--------------------------------------------------------------------------------------
 * TODO:
 *  o Check TCP send speed. Problem seems to be in PSOCK_SEND_STR() or in uip clocks
 *  o Decide the number of DMX channels and slots in dmx.h
 *  o Clean up the code
 *  o Use a flash block to save data and configuration (probably Block 7 0x7000-0x7FFF
 * 
 * 
 *-------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "LPC11xx.h"    // LPC11xx definitions
#include "IAP.h"
#include "common.h"     // Common definitions for all project
#include "uart.h"       // RS232
#include "timer.h"      // uip timer
#include "connection.h" // TCP incoming data handling
#include "uip.h"
#include "uip_arp.h"
#include "dm9k_uip.h"   // DM9000C driver
#include "dmx.h"        // DMX driver
#include "sACN.h"
#include "wwdt.h"

#define DEBUG(X) TRACEL(TRACE_LEVEL_GENERIC, X)

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])     // Global uip data buffer for incoming/outcoming data
#define TCP_BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])     // Global uip data buffer for incoming/outcoming data

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

tp_glb glb;

// Function prototypes
//
void    UART_Handler(void);
void    Init_hw(void);
int     atoi2(const char *s);
int     xtoi(const char *xs, uint8_t *result);

uint8_t flash_erase(void);
uint8_t flash_save(void);
void    flash_load(void);
void    flash_restore_defaults(void);


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main(void) {

    struct timer periodic_timer, arp_timer;

    // Initialise hardware
    Init_hw();
    
    /* Config WDT clock */
    //WDTInit();

    // Get functioning mode.
    // If 0: then RS485 and DMX is enabled (250Kbps), if 1: then RS232 trace/command mode is enabled (115kbps)
    // RS485 handler is DMX_Handler() in dmx.c (also the tmr32_0 interrupt routine
    // RS232 handler is UART_Handler() in main.c
    // To select mode use jumber in GND pos for RS485 mode or in CFG pos for RS232 mode
    //
    glb.debug = (LPC_GPIO1->DATA & (1 << PIN_CFG)) ? 0 : 1;

    if (glb.debug) {    // RS232 mode
        UART_Init(115200);
        // Set trace levels
        glb.trace_levels =
            TRACE_LEVEL_GENERIC |
//              TRACE_LEVEL_DM9K |
//              TRACE_LEVEL_UIP |
//              TRACE_LEVEL_RGB |
//              TRACE_LEVEL_DMX |
//              TRACE_LEVEL_ARTNET |
//              TRACE_LEVEL_UDP |
//              TRACE_LEVEL_TCP |
                0
        ;
        LPC_GPIO1->DATA &= ~(1 << PIN_485_EN);

    } else {  // RS485 mode
        tp_dmx_rgb rgb;

        DMX_Init();

        glb.trace_levels = 0;
        DMX_GetChannelRGB(1, &rgb); // Retrieve channel 1 rgb pointer
        rgb.R = 0;  // Reset colors
        rgb.G = 0;
        rgb.B = 0;
        //DMX_SetChannel(1, rgb.R, rgb.G, rgb.B); // Update colors
        // or
        DMX_SetChannelRGB(1, &rgb);
        LPC_GPIO1->DATA |= 1 << PIN_485_EN;
    }

    DEBUG(("System core clock = %u\r\n", SystemCoreClock));

    clock_init();   // Initialize tick clock used in uip. 10ms.
    timer_set(&periodic_timer, CLOCK_SECOND / 2);   // set uip clocks
    timer_set(&arp_timer, CLOCK_SECOND * 10);

    flash_load();

    acn_init();

    etherdev_set_mac_address((uint8_t *)glb.conf.uip_ethaddr.addr);

    etherdev_init();    // Initialize DM9000C driver
    init_ip();          // Initialize all uip and IP things

    DEBUG(("Starting main loop\r\n"));  // Oh really?

    glb.reinit = 0;

    while (1) {

        if (glb.reinit) {   // is somebody asks for uip re-init??
            glb.reinit = 0;
            init_ip();
            flash_save();
        }

        if (glb.debug) UART_Handler();
        else DMX_Handler();

        uip_len = etherdev_poll();  // poll DM9000C for incoming data
        if ((uip_len > 0) && (TCP_BUF->proto != UIP_PROTO_UDP)) {
            //DEBUG(("TCP len: %d, type: 0x%04X\r\n", uip_len, htons(BUF->type) ));

            if (BUF->type == htons(UIP_ETHTYPE_IP)) {   // TCP requests

                uip_arp_ipin(); // update ARP table
                uip_input();    // let uip lib handle the incoming data
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {

                    uip_arp_out();
                    DEBUG(("send data... %d\r\n", uip_len));
                    etherdev_send();
                }
            } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {   // ARP requests

                uip_arp_arpin();
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    DEBUG(("send data... %d\r\n", uip_len));
                    etherdev_send();
                }
            }
        } else if ((uip_len > 0) && (TCP_BUF->proto == UIP_PROTO_UDP)) {
            //DEBUG(("UDP len: %d, type: 0x%04X\r\n", uip_len, htons(BUF->type) ));

            if (BUF->type == htons(UIP_ETHTYPE_IP)) {   // TCP requests
                uip_input();    // let uip lib handle the incoming data
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    //uip_arp_out();
                    etherdev_send();
                }
            }
        } else if (timer_expired(&periodic_timer)) {
            uint8_t i;

            timer_reset(&periodic_timer);
            //
            for (i = 0; i < UIP_CONNS; i++) {
                uip_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    etherdev_send();
                }
            }

            for (i = 0; i < UIP_UDP_CONNS; i++) {
                uip_udp_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    etherdev_send();
                }
            }


            /* Call the ARP timer function every 10 seconds. */
            if (timer_expired(&arp_timer)) {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }

        if (glb.ticks >= 99) {                  /* Set Clock1s to 1 every 1 second    */
            glb.ticks = 0;
            //DEBUG(("tick: %u\r\n", clock_time()));
        }
    }
}


void uip_log(char *msg) {
    DEBUG(("uip_log: %s\r\n", msg));
}

void init_ip(void)
{
    uip_close();
    uip_init();         // Initialize uip
    // Set ip address
    uip_sethostaddr(glb.conf.uip_ipaddr);
    // Set default gateway
    uip_setdraddr(glb.conf.uip_gateway);
    // Set default subnet mask
    uip_setnetmask(glb.conf.uip_subnet);
    // Set default MAC to uIP
    uip_setethaddr(glb.conf.uip_ethaddr);
    connection_init();  // Initialize uip socket. In connection.c is YOUR socket functions
    udp_connection_init();
}



void UART_Handler(void) {
    if (glb_uart_buffer.rx_timeSinceLastByte > 2) {
        char *inCmd = (char *)glb_uart_buffer.rxBuffer;
        uint16_t len = glb_uart_buffer.rx_index;

        glb_uart_buffer.rx_timeSinceLastByte = 0;
        glb_uart_buffer.rx_flag = 0;    //reset read flag
        glb_uart_buffer.rx_index = 0;

        if (len >= 1) {    // Commands must be at least 17 chars long
            inCmd[len] = 0; // terminate the string
            if (!strncmp(inCmd, "HELP", 4)) {
                // TODO: complete help....
                DEBUG(("Supported Commands: \r\n"));
                DEBUG(("\t[HELP]: This menu\r\n"));
                DEBUG(("\t[TRACE?/=]: Prints/Sets current trace levels\r\n"));
                DEBUG(("\t[HOSTIP?/=]: Gets/Sets MAC address\r\n"));
                DEBUG(("\t[MAC?/=]: Gets/Sets MAC address\r\n"));
                DEBUG(("\t[GATEWAY?/=]: Gets/Sets default gateway\r\n"));
                DEBUG(("\t[SUBNET?/=]: Gets/Sets subnet mask\r\n"));
                DEBUG(("\t[RATE?/=]: Gets/Sets color change rate 0: instant, >1: slower to fast\r\n"));
                DEBUG(("\t[TCPMODE?/=]: TCP mode. 0: ASCII, 1: Binary\r\n"));
                DEBUG(("\t[SAVE]: Save current configuration to flash\r\n"));
                DEBUG(("\t[RESTORE]: Restore default configuration\r\n"));
                DEBUG(("\t[RESET]: Perform software reset on DM9000\r\n"));
            }
            // TRACE levels
            else if (!strncmp(inCmd, "TRACE", 5)) {
                int tmpLevel;
                char tmpstr[10];

                if (inCmd[5] == '?') {
                    DEBUG(("Current trace level: %d\r\n", glb.trace_levels));
                } else {
                    strcpy(tmpstr, inCmd + 6);
                    tmpLevel = atoi2(tmpstr);

                    if (tmpLevel == 0) glb.trace_levels = 0;
                    else glb.trace_levels |= tmpLevel;

                    DEBUG(("Adding trace for %d \r\n", tmpLevel));
                }
            }
            // MAC address
            else if (!strncmp(inCmd, "MAC", 3)) {
                if (inCmd[3] == '?') {  // Get MAC address
                    DEBUG(("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                           glb.conf.uip_ethaddr.addr[0], glb.conf.uip_ethaddr.addr[1], glb.conf.uip_ethaddr.addr[2],
                           glb.conf.uip_ethaddr.addr[3], glb.conf.uip_ethaddr.addr[4], glb.conf.uip_ethaddr.addr[5]));
                } else if (inCmd[3] == '=') { // Set MAC address
                    char *pch;
                    uint8_t i = 0;
                    DEBUG(("MAC from: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                           glb.conf.uip_ethaddr.addr[0], glb.conf.uip_ethaddr.addr[1], glb.conf.uip_ethaddr.addr[2],
                           glb.conf.uip_ethaddr.addr[3], glb.conf.uip_ethaddr.addr[4], glb.conf.uip_ethaddr.addr[5]));

                    pch = strtok(&inCmd[4], ":");
                    while (pch != NULL) {
                        xtoi(pch, &glb.conf.uip_ethaddr.addr[i++]);
                        pch = strtok(NULL, ":");
                    }

                    uip_setethaddr(glb.conf.uip_ethaddr);
                    etherdev_set_mac_address((uint8_t *)glb.conf.uip_ethaddr.addr);
                    //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                    flash_save();

                    DEBUG((" is set to: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                           glb.conf.uip_ethaddr.addr[0], glb.conf.uip_ethaddr.addr[1], glb.conf.uip_ethaddr.addr[2],
                           glb.conf.uip_ethaddr.addr[3], glb.conf.uip_ethaddr.addr[4], glb.conf.uip_ethaddr.addr[5]));
                }
            }
            // Local IP address
            else if (!strncmp(inCmd, "HOSTIP", 6)) {
                if (inCmd[6] == '?') {  // Get ip address
                    DEBUG(("Host IP: %d.%d.%d.%d\r\n",
                           glb.conf.uip_ipaddr[0] & 0xFF, glb.conf.uip_ipaddr[0] >> 8,
                           glb.conf.uip_ipaddr[1] & 0xFF, glb.conf.uip_ipaddr[1] >> 8));
                } else if (inCmd[6] == '=') { // Set ip address
                    char *pch;
                    uint8_t addr[4];
                    uint8_t i = 0;
                    DEBUG(("IP from: %d.%d.%d.%d",
                           glb.conf.uip_ipaddr[0] & 0xFF, glb.conf.uip_ipaddr[0] >> 8,
                           glb.conf.uip_ipaddr[1] & 0xFF, glb.conf.uip_ipaddr[1] >> 8));

                    pch = strtok(&inCmd[7], ".");
                    while (pch != NULL) {
                        addr[i++] = atoi2(pch);
                        pch = strtok(NULL, ".");
                    }
                    uip_ipaddr(glb.conf.uip_ipaddr, addr[0], addr[1], addr[2], addr[3]);
                    uip_sethostaddr(glb.conf.uip_ipaddr);
                    //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                    flash_save();

                    DEBUG((" is set to: %d.%d.%d.%d\r\n",
                           glb.conf.uip_ipaddr[0] & 0xFF, glb.conf.uip_ipaddr[0] >> 8,
                           glb.conf.uip_ipaddr[1] & 0xFF, glb.conf.uip_ipaddr[1] >> 8));
                }
            }
            // Gateway
            else if (!strncmp(inCmd, "GATEWAY", 7)) {
                if (inCmd[7] == '?') {  // Get ip address
                    DEBUG(("Gateway: %d.%d.%d.%d\r\n",
                           glb.conf.uip_gateway[0] & 0xFF, glb.conf.uip_gateway[0] >> 8,
                           glb.conf.uip_gateway[1] & 0xFF, glb.conf.uip_gateway[1] >> 8));
                } else if (inCmd[7] == '=') { // Set ip address
                    char *pch;
                    uint8_t addr[4];
                    uint8_t i = 0;
                    DEBUG(("Gateway from: %d.%d.%d.%d",
                           glb.conf.uip_gateway[0] & 0xFF, glb.conf.uip_gateway[0] >> 8,
                           glb.conf.uip_gateway[1] & 0xFF, glb.conf.uip_gateway[1] >> 8));

                    pch = strtok(&inCmd[8], ".");
                    while (pch != NULL) {
                        addr[i++] = atoi2(pch);
                        pch = strtok(NULL, ".");
                    }
                    uip_ipaddr(glb.conf.uip_gateway, addr[0], addr[1], addr[2], addr[3]);
                    uip_setdraddr(glb.conf.uip_gateway);
                    //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                    flash_save();

                    DEBUG((" is set to: %d.%d.%d.%d\r\n",
                           glb.conf.uip_gateway[0] & 0xFF, glb.conf.uip_gateway[0] >> 8,
                           glb.conf.uip_gateway[1] & 0xFF, glb.conf.uip_gateway[1] >> 8));
                }
            }
            // Submask
            else if (!strncmp(inCmd, "SUBNET", 6)) {
                if (inCmd[6] == '?') {  // Get ip address
                    DEBUG(("Subnet: %d.%d.%d.%d\r\n",
                           glb.conf.uip_subnet[0] & 0xFF, glb.conf.uip_subnet[0] >> 8,
                           glb.conf.uip_subnet[1] & 0xFF, glb.conf.uip_subnet[1] >> 8));
                } else if (inCmd[6] == '=') { // Set ip address
                    char *pch;
                    uint8_t addr[4];
                    uint8_t i = 0;
                    DEBUG(("Subnet from: %d.%d.%d.%d",
                           glb.conf.uip_subnet[0] & 0xFF, glb.conf.uip_subnet[0] >> 8,
                           glb.conf.uip_subnet[1] & 0xFF, glb.conf.uip_subnet[1] >> 8));

                    pch = strtok(&inCmd[7], ".");
                    while (pch != NULL) {
                        addr[i++] = atoi2(pch);
                        pch = strtok(NULL, ".");
                    }
                    uip_ipaddr(glb.conf.uip_subnet, addr[0], addr[1], addr[2], addr[3]);
                    uip_setnetmask(glb.conf.uip_subnet);
                    //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                    flash_save();

                    DEBUG((" is set to: %d.%d.%d.%d\r\n",
                           glb.conf.uip_subnet[0] & 0xFF, glb.conf.uip_subnet[0] >> 8,
                           glb.conf.uip_subnet[1] & 0xFF, glb.conf.uip_subnet[1] >> 8));
                }
            }
            // DMX rate
            else if (!strncmp(inCmd, "RATE", 4)) {
                if (inCmd[4] == '?') {  // Get dmx speed rate
                    DEBUG(("DMX Rate: %d\r\b", glb.conf.dmx_rate));
                } else if (inCmd[4] == '=') {
                    uint8_t new_rate = atoi2(&inCmd[5]);
                    DEBUG(("DMX rate changed from %d to %d\r\n", glb.conf.dmx_rate, new_rate));
                    glb.conf.dmx_rate = new_rate;
                }
            } else if (!strncmp(inCmd, "TCPMODE", 7)) {
                if (inCmd[7] == '?') {  // Get dmx speed rate
                    DEBUG(("DMX TCP mode: %d\r\b", glb.conf.dmx_tcp_mode));
                } else if (inCmd[7] == '=') {
                    uint8_t tcp_mode = atoi2(&inCmd[8]);
                    DEBUG(("DMX rate changed from %d to %d\r\n", glb.conf.dmx_tcp_mode, tcp_mode));
                    glb.conf.dmx_tcp_mode = tcp_mode;
                }
            }
            // Flash save
            else if (!strncmp(inCmd, "SAVE", 4)) {
                DEBUG(("Configuration saved.\r\n"));
                //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                flash_save();
            }
            // Flash restore to defaults
            else if (!strncmp(inCmd, "RESTORE", 7)) {
                DEBUG(("Restored to defaults...\r\n"));
                flash_restore_defaults();
            }
            // Reset DM9000
            else if (!strncmp(inCmd, "RESET", 5)) {
                dm9k_reset();
                DEBUG(("DM9000 reset\r\n"));
            }


        } //:~ Valid command
    }
}


/**
 * Initializes LPC-1114/301
 * 
 * @author jaco (10/11/2014)
 */
void Init_hw(void) {
#define INACTIVE    0x00
#define PULL_UP     0x02
#define PULL_DOWN   0x01
#define PIN_PULL    PULL_UP

    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6) | (1 << 16);

    LPC_IOCON->PIO3_4 |= (PIN_PULL << 3);    // SD0 pull-up
    LPC_IOCON->PIO3_5 |= (PIN_PULL << 3);    // SD1 pull-up
    LPC_IOCON->PIO0_6 |= (PIN_PULL << 3);    // SD2 pull-up
    LPC_IOCON->PIO0_7 |= (PIN_PULL << 3);    // SD3 pull-up
    LPC_IOCON->PIO0_8 |= (PIN_PULL << 3);    // SD4 pull-up
    LPC_IOCON->PIO0_9 |= (PIN_PULL << 3);    // SD5 pull-up
    LPC_IOCON->SWCLK_PIO0_10 |= (PIN_PULL << 3) | (1 << 0);    // SD6 pull-up and GPIO function
    LPC_IOCON->R_PIO0_11 |= (PIN_PULL << 3) | (1 << 0);        // SD7 pull-up and GPIO function

    LPC_IOCON->PIO0_2 |= (PULL_UP << 3);    // IOW
    LPC_IOCON->PIO0_3 |= (PULL_UP << 3);    // IOR
    LPC_IOCON->PIO2_0 |= (PULL_UP << 3);    // DEBUG PIN
    LPC_IOCON->R_PIO1_1 |= (PULL_UP << 3) | (1 << 0);    // RS485_EN
    LPC_IOCON->SWDIO_PIO1_3 |= (PULL_UP << 3) | (1 << 0); // CFG select
    LPC_IOCON->PIO1_8 |= (PULL_UP << 3);    // PWRST
    LPC_IOCON->PIO1_9 |= (PULL_UP << 3);    // CMD

    // All data pins are outputs
    LPC_GPIO0->DIR = DATA_PORT0_OUT;    // SD2-SD7=Out
    LPC_GPIO3->DIR = DATA_PORT3_OUT;    // SD0-SD1=Out
    LPC_GPIO1->DIR = 0x0382;            // CMD=Out, PWRST=Out, TX=Out, RX=Inp, RS485_EN=Out
    LPC_GPIO2->DIR |= (1 << 0);         // Debug pin is output

    LPC_GPIO0->DATA = 0x000C;   // IOR=1, IOW=1, SD7-SD2=0
    LPC_GPIO3->DATA = 0;        // SD1-SD0=0
    LPC_GPIO1->DATA = 0x0382;   // CMD=1, PWRST=1, TX=1
    LPC_GPIO2->DATA = ~(1 << 0);
}

/**
 * @brief   Convert string to integer
 * 
 * @author dtassopoulos (28/5/2013)
 * 
 * @param s String to be converted
 * 
 * @return int  Integer after convertion
 */
int atoi2(const char *s) {
    static const char digits[] = "0123456789";  /* legal digits in order */
    unsigned val = 0;         /* value we're accumulating */
    int neg = 0;              /* set to true if we see a minus sign */

    /* skip whitespace */
    while (*s == ' ' || *s == '\t') {
        s++;
    }

    /* check for sign */
    if (*s == '-') {
        neg = 1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    /* process each digit */
    while (*s) {
        const char *where;
        unsigned digit;

        /* look for the digit in the list of digits */
        where = strchr(digits, *s);
        if (where == NULL) {
            /* not found; not a digit, so stop */
            break;
        }

        /* get the index into the digit list, which is the value */
        digit = (where - digits);

        /* could (should?) check for overflow here */

        /* shift the number over and add in the new digit */
        val = val * 10 + digit;

        /* look at the next character */
        s++;
    }

    /* handle negative numbers */
    if (neg) {
        return -val;
    }

    /* done */
    return val;
}


/**
 * Convert HEX string to byte array
 * 
 * @author jaco (10/11/2014)
 * 
 * @param xs HEX string
 * @param result The byte array
 * 
 * @return int Number of bytes found
 */
int xtoi(const char *xs, uint8_t *result) {
    size_t szlen = strlen(xs);
    int i, xv, fact;

    if (szlen > 0) {
        // Converting more than 32bit hexadecimal value?
        if (szlen > 8) return 2; // exit

        // Begin conversion here
        *result = 0;
        fact = 1;

        // Run until no more character to convert
        for (i = szlen - 1; i >= 0; i--) {
            if (*(xs + i) >= 97) {
                xv = (*(xs + i) - 97) + 10;
            } else if (*(xs + i) >= 65) {
                xv = (*(xs + i) - 65) + 10;
            } else {
                xv = *(xs + i) - 48;
            }
            *result += (xv * fact);
            fact *= 16;
        }
    }
    return (1);
}

#define IAP_LOCATION        0x1FFF1FF1
#define FLASH_START_ADDR    0x007000UL
#define FLASH_END_ADDR	    0x008000UL
#define FLASH_HEADER        0x0ABCDEF0  // the flash must always start with this
#define FLASH_SECTOR        7   // use the last 4K flash 0x00007000 - 0x00007FFF


void flash_restore_defaults(void)
{
    uip_close();

    // Set flash header
    glb.conf.header = FLASH_HEADER;
    // Set flash version
    glb.conf.version = FLASH_VERSION;

    // Set default ip
    uip_ipaddr(glb.conf.uip_ipaddr, 192, 168, 1, 234);
    // Set default gateway
    uip_ipaddr(glb.conf.uip_gateway, 192, 168, 1, 1);
    // Set default subnet
    uip_ipaddr(glb.conf.uip_subnet, 255, 255, 255, 0);

    // Set default MAC address
    glb.conf.uip_ethaddr.addr[0] = 0x10; glb.conf.uip_ethaddr.addr[1] = 0x20; glb.conf.uip_ethaddr.addr[2] = 0x30;
    glb.conf.uip_ethaddr.addr[3] = 0x40; glb.conf.uip_ethaddr.addr[4] = 0x50; glb.conf.uip_ethaddr.addr[5] = 0x60;
	
    // Set default DMX/TCP mode
    glb.conf.dmx_tcp_mode = DMX_TCP_MODE_ASCII;
    glb.conf.dmx_rate = 0;
    glb.conf.dmx_slots = DMX_SLOT_NUM;
    glb.conf.dmx_universe = 1;

    glb.conf.acn_port = ACN_DEF_UDP_PORT;
	
	// Load Art-Net defaults
    artnet_defaults();

    //flash_save();

    uip_init();             // Initialize uip
    connection_init();      // re-init TCP
    udp_connection_init();  // re-init UDP

}

uint8_t flash_erase(void) {
    uint8_t resp = 0;

    if (u32IAP_PrepareSectors(FLASH_SECTOR, FLASH_SECTOR) == IAP_STA_CMD_SUCCESS) {
        if (u32IAP_EraseSectors(FLASH_SECTOR, FLASH_SECTOR) == IAP_STA_CMD_SUCCESS) {

            resp = 1;
        }
    }
    return(resp);
}


uint8_t flash_save(void) {
    uint8_t result = 0;
    uint32_t u32NextFlashWriteAddr = FLASH_START_ADDR;

    __disable_irq();
    // First erase sector
    DEBUG(("Erasing flash...\r\n"));
    if (flash_erase()) {
        DEBUG(("Prepare for writing...\r'n"));
        // Prepare the flash application sectors for reprogramming
        if (u32IAP_PrepareSectors(FLASH_SECTOR, FLASH_SECTOR) == IAP_STA_CMD_SUCCESS) {
            uint16_t datalen;

            // Write conf data
            DEBUG(("Writing config to flash\r\n"));
            datalen = sizeof(tp_conf);

            // Ensure that amount of data written to flash is at minimum the size of a flash page
            if (datalen < IAP_FLASH_PAGE_SIZE_BYTES) datalen = IAP_FLASH_PAGE_SIZE_BYTES;
            
            // Write the data to flash
            if (u32IAP_CopyRAMToFlash(u32NextFlashWriteAddr, (uint32_t) &glb.conf, datalen) == IAP_STA_CMD_SUCCESS) {
                // Check that the write was successful
                if (u32IAP_Compare(u32NextFlashWriteAddr, (uint32_t) &glb.conf, datalen, 0) == IAP_STA_CMD_SUCCESS) {
                    // Write was successful
                    DEBUG(("Flash success...\r\n"));
                    result = 1;
                }
            }
        }
    }
    else {
        DEBUG(("Erasing flash failed...\r\n"));
    }
    __enable_irq();

    return(result);
}


void flash_load(void) {
    
    memcpy( (void*) &glb.conf, (void*) (FLASH_START_ADDR), sizeof(tp_conf) );

    // check if flash is valid
    if (glb.conf.header != FLASH_HEADER) {
        // load defaults
        DEBUG(("Not valid flash data found. Restoring defaults...\r\n"));
        flash_restore_defaults();
        //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
        flash_save();
    }

    // Check that version is right
    if (glb.conf.version != FLASH_VERSION) {
        // Do necessary things or load the defaults
        DEBUG(("Wrong flash version found: %d / %d\r\n", glb.conf.version, FLASH_VERSION));

        DEBUG(("Restoring default configuration...\r\n"));
        flash_restore_defaults();
        //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
        flash_save();
    }

    DEBUG(("header: %04X %04X\r\n", glb.conf.header >> 16, glb.conf.header & 0xFFFF));
    DEBUG(("version: %d\r\n", glb.conf.version));
    DEBUG(("ip: %d.%d.%d.%d\r\n", glb.conf.uip_ipaddr[0] & 0xFF, glb.conf.uip_ipaddr[0] >> 8, 
           glb.conf.uip_ipaddr[1] & 0xFF, glb.conf.uip_ipaddr[1] >> 8 ));
    DEBUG(("gw: %d.%d.%d.%d\r\n", glb.conf.uip_gateway[0] & 0xFF, glb.conf.uip_gateway[0] >> 8, 
           glb.conf.uip_gateway[1] & 0xFF, glb.conf.uip_gateway[1] >> 8 ));
    DEBUG(("sn: %d.%d.%d.%d\r\n", glb.conf.uip_subnet[0] & 0xFF, glb.conf.uip_subnet[0] >> 8, 
           glb.conf.uip_subnet[1] & 0xFF, glb.conf.uip_subnet[1] >> 8 ));
    DEBUG(("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
           glb.conf.uip_ethaddr.addr[0], glb.conf.uip_ethaddr.addr[1], glb.conf.uip_ethaddr.addr[2],
           glb.conf.uip_ethaddr.addr[3], glb.conf.uip_ethaddr.addr[4], glb.conf.uip_ethaddr.addr[5] ));

    DEBUG(("DMX slots: %d\r\n", glb.conf.dmx_slots));
    DEBUG(("DMX rate: %d\r\n", glb.conf.dmx_rate));
    DEBUG(("DMX tcp mode: %d\r\n", glb.conf.dmx_tcp_mode));
    DEBUG(("DMX universe: %d\r\n", glb.conf.dmx_universe));

}

