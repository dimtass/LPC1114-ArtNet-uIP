/*==============================================================================
|                     dm9k_uip.c: Version 0.04 06/13/2006                      |
|                                                                              |
|          Davicom DM9000A uP NIC fast Ethernet driver for uIP.                |
|                                                                              |
|    This program is free software; you can redistribute it and/or modify it   |
|under the terms of the GNU General Public License as published by the Free    |
|Software Foundation; either version 2 of the License, or (at your option) any |
|later version.                                                                |
|                                                                              |
| This program is distributed in the hope that it will be useful,but WITHOUT   |
|ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS |
|FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.|
|                                                                              |
|    (C)Copyright 1997-1998 DAVICOM Semiconductor,Inc. All Rights Reserved.    |
|============================================================================= |
| China Agent:                        |||¤j³°¥N²z°S?G                          |
|   AXW Technology Limited.           |||  ·R?Y¤e¬µ§?¦³­­¤½¥q                  |
|   TEL : +86 (755) 8368-8556         |||  ?q???G+86 (755) 8368-8556           |
|         +86 (755) 8368-8967         |||        +86 (755) 8368-8967           |
|   FAX : +86 (755) 8377-8765         |||  ¶??u?G+86 (755) 8377-8765           |
|   MAIL: sales@axwdragon.com         |||  ¶l?q?Gsales@axwdragon.com           |
==============================================================================*/
/*=============================================================================
  DM9000A ¨t²?³]?m§»
  =============================================================================*/

#define DM9000A_FLOW_CONTROL
//#define DM9000A_UPTO_100M
#define Fifo_Point_Check
#define Point_Error_Reset
//#define Fix_Note_Address

#include "dm9k_uip.h"
#include "uip.h"

#define DEBUG(X)   TRACEL(TRACE_LEVEL_DM9K, X)

/*=============================================================================
  DM9000A ±µ¦¬¨???³]?m§»
  =============================================================================*/
//#define Rx_Int_enable
#define Max_Int_Count			1
#define Max_Ethernet_Lenth		1536
#define Broadcast_Jump
#define Max_Broadcast_Lenth		500

/*=============================================================================
  DM9000A ¶?°e¨???³]?m§»
  =============================================================================*/
#define Max_Send_Pack			2

/*=============================================================================
  ¨t²?¥?°µ????¶q
  =============================================================================*/
#define ETH_ADDR_LEN			6

#define DM9000_IO      0
#define DM9000_DATA    1

#define IOR_0   LPC_GPIO0->DATA &= ~(1<<PIN_IOR)
#define IOR_1   LPC_GPIO0->DATA |= (1<<PIN_IOR)
#define IOW_0   LPC_GPIO0->DATA &= ~(1<<PIN_IOW)
#define IOW_1   LPC_GPIO0->DATA |= (1<<PIN_IOW)
#define CMD_0   LPC_GPIO1->DATA &= ~(1<<PIN_CMD)
#define CMD_1   LPC_GPIO1->DATA |= (1<<PIN_CMD)
#define PWRST_0 LPC_GPIO1->DATA &= ~(1<<PIN_PWRST)
#define PWRST_1 LPC_GPIO1->DATA |= (1<<PIN_PWRST)

#define IO_CMD(VAL) if (VAL) CMD_1; else CMD_0
#define IO_PWRST(VAL) if (VAL) PWRST_1; else PWRST_0

static unsigned char DEF_MAC_ADDR[ETH_ADDR_LEN] =
{ UIP_ETHADDR0, UIP_ETHADDR1, UIP_ETHADDR2, UIP_ETHADDR3, UIP_ETHADDR4, UIP_ETHADDR5 };
uint8_t  SendPackOk = 0;

// Functions declarations
void InitNetGpio(void);
void dm9k_hash_table(void);
//void dm9k_reset(void);
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata);
uint16_t dm9k_receive_packet(void);

#define PORT0_DATA_MASK 0x00FC
#define PORT3_DATA_MASK 0x0003
#define PORT0_DATA_POS  6       // data starts from P0.6
#define PORT3_DATA_POS  4       // data starts from P3.4


/**
 * @brief 1ìsec delay
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param time 
 */
void dm9k_udelay(uint16_t usec) {
    usec = (usec * 43)/10;
    //time = time / 10;
    while (usec--);
}


/**
 * @brief Output interface routine
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param d Data
 * @param r 0: Index port, 1: Data port
 */
void DM9000_outb(uint8_t d, uint8_t r)   {

    uint32_t data = (uint32_t)(0x00FF & d);
    uint32_t data1 = 0;
    uint32_t data2 = 0;

    IO_CMD(r);

    // Set data port to output
    LPC_GPIO0->DIR = DATA_PORT0_OUT;
    LPC_GPIO3->DIR = DATA_PORT3_OUT;
    LPC_GPIO0->DATA = (1 << PIN_IOR) | (1 << PIN_IOW);    // set IOR, IOW

    // Load data
    data1 = (data & PORT3_DATA_MASK) << PORT3_DATA_POS;
    data2 = (data & PORT0_DATA_MASK) << 4;

    LPC_GPIO3->DATA = data1;
    LPC_GPIO0->DATA |= data2;   // load data and also set IOR and IOW to 1

    //DEBUG(("w: %04X %04X %04X\r\n", data2, data1, data));

    IOW_0;  // start write
            // Here the data are sent
    IOW_1;  // start write
}


/**
 * @brief Write interface routine
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param r 0: index port, 1: data port
 * 
 * @return unsigned char Input data
 */
unsigned char DM9000_inb(uint8_t r)   {
    uint32_t data = 0;
    uint32_t data1 = 0;
    uint32_t data2 = 0;

    IO_CMD(r);  // set index or data port to dm9000

    // Set data ports to input
    LPC_GPIO0->DIR = DATA_PORT0_INP;
    LPC_GPIO3->DIR = DATA_PORT3_INP;
    LPC_GPIO0->DATA = (1 << PIN_IOR) | (1 << PIN_IOW);    // set IOR, IOW

    IOR_0;  // start read
            // Here data are red
    data1 = LPC_GPIO3->DATA;
    data2 = LPC_GPIO0->DATA;

    IOR_1;  // stop read

    data = (data1 >> PORT3_DATA_POS) & PORT3_DATA_MASK;
    data |= (data2 >> 4) & PORT0_DATA_MASK;

    //if (data) DEBUG(("d: %04X %04X %04X\r\n", data2, data1, data));
    return ((unsigned char)(0xFF & data));
}


/**
 * @brief Read data from register
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param reg Register to read from
 * 
 * @return uint8_t The data of the register
 */
uint8_t ior(uint8_t reg) {
    unsigned char d = 0;
    DM9000_outb(reg, DM9000_IO);
    d = DM9000_inb(DM9000_DATA);
    return  d;
}


/**
 * @brief Write data to register
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param reg DM9000C's register
 * @param data Data to write
 */
void iow(uint8_t reg, uint8_t data) {
    DM9000_outb(reg, DM9000_IO);
    DM9000_outb(data, DM9000_DATA);
}


/**
 * @brief Initialize DM9000C
 * 
 * @author dtassopoulos (7/11/2014)
 */
void dm9k_initnic(void) {
    uint16_t i;

    DEBUG(("InitNetGpio\r\n"));
    InitNetGpio();                                  // ?? DM9000A ¶i¦?µw¥s­«?m

    
    iow(DM9000_REG_GPCR, 0x07);    // Power PHY set the GPR to output
    iow(DM9000_REG_GPR,  0x06);	  // Clear the GPIO lines (Led's off)




    DEBUG(("DM9000_REG_RESET\r\n"));
    dm9k_udelay(10);
    iow(DM9000_REG_NCR, 0x03);          // Software reset
    dm9k_udelay(40);
    iow(DM9000_REG_NCR, 0x00);          // Clear reset
    dm9k_udelay(5);
    iow(DM9000_REG_NCR, 0x03);          // Software reset
    dm9k_udelay(40);
    iow(DM9000_REG_NCR, 0x00);          // Clear reset
    dm9k_udelay(5);

    //3. Clear TX status by reading NSR register (reg_01h). Bit[2:3] and bit[5] will be automatically
    // cleared by reading or writing 1. Please refer to the DM9000 datasheet ch.6.2 about NSR
    // register setting.
    ior(DM9000_REG_NCR);          // Select mode

    

    //4. Read EEPROM (No EEPROM available - use scratchpad instead) data if
    // valid or required. By default, the array will have hard MAC loaded.

    //5. Set Node address 6 bytes from in physical address register (reg_10h~15h).DEBUG(("dm9k_hash_table\r\n"));
    
    /* ³]?m ?t¥d MAC ¦µ?m?A¨S¦?©s MyHardware */
    for (i = 0; i < 6; i++) 
        iow(DM9000_REG_PAR + i, DEF_MAC_ADDR[i]);

    //6. Set Hash table 8 bytes from multicast address register (reg_16h~1Dh).
    for (i = 0; i < 8; i++)
        iow(DM9000_REG_MAR + i, 0xff);

    //7. reset Internal PHY if desired
    dm9k_phy_write(DM9000_REG_BMCR, 0x8000);	// PHY reset
    dm9k_udelay(1000);
    dm9k_phy_write(DM9000_REG_BMCR, 0x1200);	// Auto negotiation
    dm9k_udelay(1000);


    //8. Set IMR register (reg_FFh) bit[7]=1 to enable the SRAM read/write pointer which is the
    // automatic return function of the memory R/W address pointer; also set the receive int mask.
    iow(DM9000_REG_IMR, 0x81);

    //9. Depend on OS and DDK of the system to handle NIC interrupts.
    // Not applicable since the DM9000 is polled

    //10. Program IMR register (reg_FFh) bit[1:0] to enable the TX/RX interrupt. Before doing this,
    // the system designer needs to register the interrupt handler routine. For example, if the driver
    // needs to generate the interrupt after a package is transmitted, the interrupt mask register IMR
    // bit[1]=1 will be set. If the interrupt is generated by the DM9000 after receiving a packet, IMR
    // bit[0] should be set to 1.
    // Not applicable since the DM9000 is polled

    //11. Program RXCR register to enable RX. The RX function is enabled by setting the RX control
    // register (reg_05h) bit[0]=1. The choice of the other bits bit[6:0] depends on the system design.
    // Please refer to the DM9000 datasheet ch.6.6 about RXCR register setting.
    iow(DM9000_REG_RCR, 0x31);	// Only MAC-packets and RX enable - no multicast packets
    //iow(DM9000_REG_RCR, 0x39);	// Only MAC-packets and RX enable - no multicast packets


    //12. NIC is being activated now.
    //  while (1) write_nicreg(DM9000_MAC_REG,  uip_ethaddr.addr[0]);


    //13. Check Vendor ID is Davicom (0x0A46)
//  if ((read_nicreg(DM9000_VID) + (read_nicreg(DM9000_VID+1) << 8)) != DM9000_VendID)
//  return TRUE;		// No chip found
//
//  if ((read_nicreg(DM9000_PID) + (read_nicreg(DM9000_PID+1) << 8)) != DM9000_ProdID)
//  return TRUE;		// No chip found

    DEBUG(("Init finished\r\n"));

    DEBUG(("MAC: "));
    for (i = 0; i < 6; i++) {
        char a = ior(DM9000_REG_PAR + i);
        DEBUG(("%02X ", a));
    }
    DEBUG(("\r\n"));
}


/**
 * @brief Do hardware reset to DM9000C
 * 
 * @author dtassopoulos (7/11/2014)
 */
void InitNetGpio(void) {
    uint8_t i;

    IO_PWRST(0);
    for (i = 0; i < 200; i++) {
        dm9k_udelay(1000);
    }
    IO_PWRST(1);
    for (i = 0; i < 200; i++) {
        dm9k_udelay(1000);
    }
}

void etherdev_set_mac_address(uint8_t * MAC)
{
    uint8_t i;
    DEBUG(("Setting MAC "));
    for (i=0; i<6; i++) {
        DEF_MAC_ADDR[i] = MAC[i];
        DEBUG((":%02X", DEF_MAC_ADDR[i]));
    }
    DEBUG(("\r\n"));
}

void etherdev_get_mac_address(uint8_t ** MAC)
{
    uint8_t i;
    for (i=0; i<6; i++) *MAC[i] = DEF_MAC_ADDR[i];
}


/**
 * @brief Reset DM9000C
 * 
 * @author dtassopoulos (7/11/2014)
 */
void dm9k_reset(void) {
    iow(DM9000_REG_NCR, DM9000_REG_RESET);          /* ?? DM9000A ¶i¦?³n¥s­«?m */
    dm9k_udelay(20);                                /* delay 10us */
    iow(DM9000_REG_NCR, DM9000_REG_RESET);          /* ?? DM9000A ¶i¦?³n¥s­«?m */
    dm9k_udelay(20);                                /* delay 10us */

    /* °?¥»°O¦s??¬?Gf³]?m */
    iow(DM9000_REG_IMR, DM9000_IMR_OFF);            /* ¶}±?¤?¦s¦??t??¦? */
    iow(DM9000_REG_TCR2, DM9000_TCR2_SET);          /* ³]?m LED ??¥???¦? */

    /* ²M°£¦h?l??°T */
    iow(DM9000_REG_NSR, 0x2c);
    iow(DM9000_REG_TCR, 0x00);
    iow(DM9000_REG_ISR, 0x3f);

#ifdef DM9000A_FLOW_CONTROL
    iow(DM9000_REG_BPTR, DM9000_BPTR_SET);          /* ¥b??¤u¬y±±³]?m */
    iow(DM9000_REG_FCTR, DM9000_FCTR_SET);          /* ¥???¤u¬y±±³]?m */
    iow(DM9000_REG_FCR, DM9000_FCR_SET);            /* ¶}±?¬y±±³]?m */
#endif

#ifdef DM9000A_UPTO_100M
    iow(DM9000_REG_OTCR, DM9000_OTCR_SET);          /* ¤u§@?W²v¨µ 100Mhz ³]?m */
#endif

#ifdef  Rx_Int_enable
    iow(DM9000_REG_IMR, DM9000_IMR_SET);            /* ¶}±? ¤¤?_??¦? */
#else
    iow(DM9000_REG_IMR, DM9000_IMR_OFF);            /* Gf³¬ ¤¤?_??¦? */
#endif

    iow(DM9000_REG_RCR, DM9000_RCR_SET);            /* ¶}±? ±µ¦¬¤u?? */

    SendPackOk = 0;
}


/**
 * @brief Write data to PHY register
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param phy_reg The PHY register
 * @param writedata Data to write
 */
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata) {
    /* ³]?m?g¤J PHY ±H¦s????¦µ?m */
    iow(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

    /* ³]?m?g¤J PHY ±H¦s????­T */
    iow(DM9000_REG_EPDRH, (writedata >> 8) & 0xff);
    iow(DM9000_REG_EPDRL, writedata & 0xff);

    iow(DM9000_REG_EPCR, 0x0a);                         /* ±N??®??g¤J PHY ±H¦s?? */
    while (ior(DM9000_REG_EPCR) & 0x01);                 /* ¬d?M¬O§_°?¦?µ²§t */
    iow(DM9000_REG_EPCR, 0x08);                         /* ²M°£?g¤J©R¥O */
}


/**
 * @brief This is the interrupt handler for interrupts 
 * in Interrupt Mask Register (FFh)
 * 
 * @author dtassopoulos (7/11/2014)
 */
void  dm9k_interrupt(void) {
    uint8_t  save_reg;
    uint8_t  isr_status;

    //save_reg = NET_REG_ADDR;                            /* ?T¦s©?¨?¥???¦µ?m */

    save_reg =  DM9000_inb(DM9000_DATA);
    iow(DM9000_REG_IMR, DM9000_IMR_OFF);               /* Ao³¬ DM9000A ¤¤A_ */
    isr_status = ior(DM9000_REG_ISR);                   /* ¨u±o¤¤A_²£¥I­E */

    if (isr_status & DM9000_RX_INTR)                     /* AE¬d¬O§_¬°±µ¦¬¤¤A_ */
        dm9k_receive_packet();                          /* °o¦?±µ¦¬³B²zµ{§C */

    iow(DM9000_REG_IMR, DM9000_IMR_SET);               /* ¶}±O DM9000A ¤¤A_ */
//  NET_REG_ADDR = save_reg;                            /* ¦^/_©O¨I¥I??¦i?m */
    DM9000_outb(save_reg, DM9000_DATA);
}


/**
 * @brief Receive a packet from DM9000 if there's any. 
 *  
 * Packet format: 
 * [READY:1][STATUS_LSB][STATUS_MSB][LENGTH_LSB][LENGTH_MSB][DATA...]
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @return uint16_t The packet length
 */
uint16_t dm9k_receive_packet(void)  {
    uint16_t packet_length;     // the retrieved packed length
    uint8_t  rx_int_count = 0;
    uint8_t  rx_checkbyte;
    uint16_t rx_status, rx_length;
    uint8_t  jump_packet;
    uint16_t calc_MRR;
    uint16_t i;

    do {
        packet_length = 0;
        //ReceiveData = (uint16_t *) &uip_buf[0];
        jump_packet = 0;                                // By default do not reject the packet
        ior(DM9000_REG_MRCMDX);                         // Read data from RX SRAM

        calc_MRR = (ior(DM9000_REG_MRRH) << 8) + ior(DM9000_REG_MRRL);  // Get memory read address
        rx_checkbyte = ior(DM9000_REG_MRCMDX);      // Read data from RX SRAM

        if (rx_checkbyte == DM9000_PKT_RDY) {   // If first byte = 0x01 then a new packet is received

            DM9000_outb(DM9000_REG_MRCMD, DM9000_IO);   // Read data with address auto-increment
            // Get rx status
            rx_status = DM9000_inb(DM9000_DATA) + (DM9000_inb(DM9000_DATA) << 8);
            // Get rx length
            rx_length = DM9000_inb(DM9000_DATA) + (DM9000_inb(DM9000_DATA) << 8);

            if (rx_length > Max_Ethernet_Lenth)   jump_packet = 1;

#ifdef Broadcast_Jump
            if (rx_status & 0x4000)   {
                if (rx_length > Max_Broadcast_Lenth)   jump_packet = 1;
            }
#endif

            /* ­p?a¤U¤@­O¥]??«u°w¦i , ­Y±µ¦¬?o«?¬°©_???A»Y¥[¤@?i»o°?¦r?`?C*/
            /* ­Y¬O¶W?L 0x3fff ?A«h»Y¦^AkA¶¨i 0x0c00 °_©l¦i?m */
            calc_MRR += (rx_length + 4);
            if (rx_length & 0x01) calc_MRR++;
            if (calc_MRR > 0x3fff) calc_MRR -= 0x3400;

            if (jump_packet == 0x01)   {
                // read this packet and skip it
                iow(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
                iow(DM9000_REG_MRRL, calc_MRR & 0xff);
                continue;
            }
            // Read all data
            for (i = 0; i < rx_length; i++) {
                uip_buf[i] = DM9000_inb(DM9000_DATA);
            }

            packet_length = rx_length - 4;  // sub the header length
            //DEBUG(("dm9k_len:%d\r\n", packet_length));

            rx_int_count++;                             /* ²O­p¦¬¥]¦??? */

#ifdef FifoPointCheck
            if (calc_MRR != ((ior(DM9000_REG_MRRH) << 8) + ior(DM9000_REG_MRRL)))   {
#ifdef Point_Error_Reset
                dm9k_reset();   // An error occured. Reset the chip
                return packet_length;
#endif
                iow(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
                iow(DM9000_REG_MRRL, calc_MRR & 0xff);
            }
#endif
            return packet_length;
        }
        else if (rx_checkbyte == DM9000_PKT_NORDY)   {        /* µL«E¥]»Y±µ¦¬?AA?¶} */
            packet_length = 0;
            break;
                //  iow(DM9000_REG_ISR, 0x3f);              /* ²M°£¤¤A_?e°T */
        }
        else   {
            dm9k_reset();                               /* ±µ¦¬«u°w¥X?u?A­«?m */
            packet_length = 0;
            break;
        }
    }
    while (rx_int_count < Max_Int_Count);                /* ¬O§_¶W?L³I¦h±µ¦¬«E¥]­p?? */
    return(packet_length);
}


#define SendPackMax  0x01   // maximum number of packets to send

/**
 * @brief Send data to ethernet
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @param p_char Data buffer
 * @param length Data length
 */
void dm9k_send_packet(uint8_t *p_char, uint16_t length) {
    //uint16_t length = length;
    uint16_t i;

    if (SendPackOk == SendPackMax) {
        while (ior(DM9000_REG_TCR) & DM9000_TCR_SET) {
            dm9k_udelay(5);
        }
        SendPackOk = 0;
    }
    SendPackOk++;

#ifdef FifoPointCheck
    /* ­p?ß¤U¤@­S¶?°e??«?°w¦µ , ­Y±µ¦¬??«?¬°©_???A»?¥[¤@??»t°?¦r?`?C*/
    /* ­Y¬O¶W?L 0x0bff ?A«h»?¦^?k?¶¨µ 0x0000 °_©l¦µ?m */
    calc_MWR = (ior(DM9000_REG_MWRH) << 8) + ior(DM9000_REG_MWRL);
    calc_MWR += length;
    if (length & 0x01) calc_MWR++;
    if (calc_MWR > 0x0bff) calc_MWR -= 0x0c00;
#endif

    // Set the TX length register
    iow(DM9000_REG_TXPLH, (length >> 8) & 0xff);
    iow(DM9000_REG_TXPLL, length & 0xff);

    DM9000_outb(DM9000_REG_MWCMD, DM9000_IO);   // Set the memory write command address (with auto-increment)

    //DEBUG(("out[%d]: ", SendLength));
    for (i = 0; i < length; i++) {
        //DEBUG(("%02X ", p_char[i]));
        DM9000_outb(p_char[i], DM9000_DATA); //NET_REG_DATA_word = SendData[i];
    }
    //DEBUG(("\r\n"));

    iow(DM9000_REG_TCR, DM9000_TCR_SET);                /* ¶i¦?¶?°e */

#ifdef FifoPointCheck
    if (calc_MWR != ((ior(DM9000_REG_MWRH) << 8) + ior(DM9000_REG_MWRL))) {
#ifdef Point_Error_Reset
        while (ior(DM9000_REG_TCR) & DM9000_TCR_SET) dm9k_udelay(5);
        dm9k_reset();
        return;
#endif
        iow(DM9000_REG_MWRH, (calc_MWR >> 8) & 0xff);
        iow(DM9000_REG_MWRL, calc_MWR & 0xff);
    }
#endif
    DEBUG(("sent: %d\r\n", i));
    return;
}


/**
 * @brief Initialize ethernet (uip)
 * 
 * @author dtassopoulos (7/11/2014)
 */
void etherdev_init(void) {
    dm9k_initnic();
}


/**
 * @brief Send data
 * 
 * @author dtassopoulos (7/11/2014)
 */
void etherdev_send(void) {
    dm9k_send_packet(uip_buf, uip_len);    //dm9k_send_packet(send_packet, send_len);
}


/**
 * @brief Read data from PHY
 * 
 * @author dtassopoulos (7/11/2014)
 * 
 * @return uint16_t Data length
 */
uint16_t etherdev_read(void) {
    return(dm9k_receive_packet());
}


/*
                            etherdev_poll()                           
                                                                      
  This function will read an entire IP packet into the uip_buf.       
  If it must wait for more than 0.5 seconds, it will return with      
  the return value 0. Otherwise, when a full packet has been read     
  into the uip_buf buffer, the length of the packet is returned.       */
uint16_t etherdev_poll(void) {
    uint16_t bytes_read;

    //DEBUG(("."));
    /* tick_count threshold should be 12 for 0.5 sec bail-out
        One second (24) worked better for me, but socket recycling
        is then slower. I set UIP_TIME_WAIT_TIMEOUT 60 in uipopt.h
        to counter this. Retransmission timing etc. is affected also. */
    while ((!(bytes_read = etherdev_read())) && (glb.dm9k_tick_count < 2)) continue;   // 49

    //DEBUG(("/\r\n"));
    //timer0_reset();
    glb.dm9k_tick_count = 0;

    return bytes_read;
}


/**
 * @brief Check DM9000C
 * 
 * @author dtassopoulos (7/11/2014)
 */
void etherdev_chkmedia(void) {
    //uint8_t status;

    while (!(ior(DM9000_REG_NSR) & DM9000_PHY)) {
        //dm9k_udelay(2000);
    }
}


/*=============================================================================
  °£?????? ??§@¨???
  =============================================================================*/
/*-----------------------------------------------------------------------------
  °£??????¨???
  -----------------------------------------------------------------------------
  ?p¦^         µL
  -----------------------------------------------------------------------------*/
//void dm9k_debug_test(void) {
//    uint32_t check_device;
//    uint8_t  check_iomode;
//    uint8_t  check_reg_fail = 0;
//    uint8_t  check_fifo_fail = 0;
//    uint16_t i;
//    uint16_t j;
//
//    iow(DM9000_REG_NCR, DM9000_REG_RESET);          /* ?? DM9000A ¶i¦?³n¥s­«?m */
//    dm9k_udelay(10);                                /* delay 10us */
//    iow(DM9000_REG_NCR, DM9000_REG_RESET);          /* ?? DM9000A ¶i¦?³n¥s­«?m */
//    dm9k_udelay(10);                                /* delay 10us */
//
//    check_device  = ior(DM9000_REG_VID_L);
//    check_device |= ior(DM9000_REG_VID_H) << 8;
//    check_device |= ior(DM9000_REG_PID_L) << 16;
//    check_device |= ior(DM9000_REG_PID_H) << 24;
//
//    if (check_device != DM9000_ID) {
//        DEBUG(("DM9K_DEBUG ==> DEIVCE NOT FOUND, SYSTEM HOLD !!\n"));
//        while (1);
//    } else {
//        DEBUG(("DM9K_DEBUG ==> DEIVCE FOUND !!\n"));
//    }
//
//    check_iomode = ior(DM9000_REG_ISR) >> 6;
//    if (check_iomode != DM9000_WORD_MODE) {
//        DEBUG(("DM9K_DEBUG ==> DEIVCE NOT WORD MODE, SYSTEM HOLD !!\n"));
//        while (1);
//    } else {
//        DEBUG(("DM9K_DEBUG ==> DEIVCE IS WORD MODE !!\n"));
//    }
//
//    DEBUG(("DM9K_DEBUG ==> REGISTER R/W TEST !!\n"));
//    //NET_REG_ADDR = DM9000_REG_MAR;
//    DM9000_outb(DM9000_REG_MAR, DM9000_IO);
////  for (i = 0; i < 0x0100; i++) {
////      NET_REG_DATA_byte = i;
////      if (i != NET_REG_DATA_byte) {
////          DEBUG(("             > error W %02x , R %02x \n", i , NET_REG_DATA_byte));
////          check_reg_fail = 1;
////      }
////  }
//
//    if (check_reg_fail) {
//        DEBUG(("DM9K_DEBUG ==> REGISTER R/W FAIL, SYSTEM HOLD !!\n"));
//        while (1);
//    }
//
//    DEBUG(("DM9K_DEBUG ==> FIFO R/W TEST !!\n"));
//    DEBUG(("DM9K_DEBUG ==> FIFO WRITE START POINT 0x%02x%02x \n",
//           ior(DM9000_REG_MWRH), ior(DM9000_REG_MWRL)));
//
//    //NET_REG_ADDR = DM9000_REG_MWCMD;
//    DM9000_outb(DM9000_REG_MWCMD, DM9000_IO);
//    for (i = 0; i < 0x1000; i++) NET_REG_DATA_word = ((i & 0xff) * 0x0101);
//
//    DEBUG(("DM9K_DEBUG ==> FIFO WRITE END POINT 0x%02x%02x \n",
//           ior(DM9000_REG_MWRH), ior(DM9000_REG_MWRL)));
//
//    if ((ior(DM9000_REG_MWRH) != 0x20) || (ior(DM9000_REG_MWRL) != 0x00)) {
//        DEBUG(("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n"));
//        while (1);
//    }
//
//    ior(DM9000_REG_MRCMDX);
//    DEBUG(("DM9K_DEBUG ==> FIFO READ START POINT 0x%02x%02x \n",
//           ior(DM9000_REG_MRRH), ior(DM9000_REG_MRRL)));
//    ior(DM9000_REG_MRCMDX);
//
//    //NET_REG_ADDR = DM9000_REG_MWCMD;
//    DM9000_outb(DM9000_REG_MWCMD, DM9000_IO);
//    for (i = 0; i < 0x1000; i++) {
//        j = NET_REG_DATA_word;
//
//        if (((i & 0xff) * 0x0101) != j) {
//            //DEBUG(("             > error W %04x , R %04x \n",
//            //      ((i & 0xff) * 0x0101) , j));
//            check_fifo_fail = 1;
//        }
//    }
//
//    DEBUG(("DM9K_DEBUG ==> FIFO READ END POINT 0x%02x%02x \n",
//           ior(DM9000_REG_MRRH), ior(DM9000_REG_MRRL)));
//
//    if ((ior(DM9000_REG_MRRH) != 0x20) || (ior(DM9000_REG_MRRL) != 0x00)) {
//        DEBUG(("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n"));
//        while (1);
//    }
//
//    if (check_fifo_fail) {
//        DEBUG(("DM9K_DEBUG ==> FIFO R/W DATA FAIL, SYSTEM HOLD !!\n"));
//        while (1);
//    }
//
//    DEBUG(("DM9K_DEBUG ==> PACKET SEND & INT TEST !! \n"));
//    iow(DM9000_REG_NCR, DM9000_REG_RESET);
//    dm9k_udelay(10);
//    iow(DM9000_REG_NCR, DM9000_REG_RESET);
//    dm9k_udelay(10);
//
//    iow(DM9000_REG_IMR, DM9000_IMR_OFF | DM9000_TX_INTR);
//
//    iow(DM9000_REG_TXPLH, 0x01);
//    iow(DM9000_REG_TXPLL, 0x00);
//
//    do {
//        iow(DM9000_REG_ISR, DM9000_TX_INTR);
//        DEBUG(("DM9K_DEBUG ==> INT PIN IS OFF\n"));
//
//        NET_REG_ADDR = DM9000_REG_MWCMD;
//        for (i = 0; i < (0x0100 / 2); i++) {
//            if (i < 3) NET_REG_DATA_word = 0xffff;
//            else NET_REG_DATA_word = i * 0x0101;
//        }
//
//        DEBUG(("DM9K_DEBUG ==> PACKET IS SEND \n"));
//        iow(DM9000_REG_TCR, DM9000_TCR_SET);
//
//        while (ior(DM9000_REG_TCR) & DM9000_TCR_SET) dm9k_udelay(5);
//        if (ior(DM9000_REG_ISR) & DM9000_TX_INTR) DEBUG(("DM9K_DEBUG ==> INT PIN IS ACTIVE \n"));
//        else DEBUG(("DM9K_DEBUG ==> INT PIN IS NOT ACTIVE \n"));
//
//        for (i = 0; i < 10; i++) dm9k_udelay(1000);
//
//    }while (1);
//}




