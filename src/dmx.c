#include "common.h"
#include "dmx.h"
#include "uart.h"
#include <string.h>

#define DEBUG(X) TRACEL(TRACE_LEVEL_DMX, X)

#define DMX_BAUD_RATE       250000
#define MR0_TIME_US         100
#define DMX_TIMEOUT_MBB     MR0_TIME_US * 100   // 400 us
#define DMX_TIMEOUT_BREAK   MR0_TIME_US * 37    // 150 us
#define DMX_TIMEOUT_MAB     MR0_TIME_US * 4     // 12 us
#define DMX_TX_0            LPC_GPIO1->DATA &= ~(1<<7)
#define DMX_TX_1            LPC_GPIO1->DATA |= (1<<7)
#define DMX_EN_0            LPC_GPIO1->DATA &= ~(1<<1)
#define DMX_EN_1            LPC_GPIO1->DATA |= (1<<1)



#define TMR32_0_ENABLE  LPC_TMR32B0->TCR = 1    //Enable 32bit timer
#define TMR32_0_DISABLE LPC_TMR32B0->TCR = 0    // Disable 32bit timer
#define TM32_0_START(TIMEOUT) TMR32_0_DISABLE;\
                        LPC_TMR32B0->MR0 = TIMEOUT;\
                        TMR32_0_ENABLE


tp_dmx_handler dmx_handler;


// Function prototypes
void TMR32_0_Init(void);


void DMX_Init(void) {
    int k = 0;
    for (k = 0; k < DMX_SLOT_NUM; k++) dmx_handler.data[k] = dmx_handler.wanted_data[k] = 0;
    glb.conf.dmx_rate = 0;

    // init uart
    UART_Init(DMX_BAUD_RATE);
    // Tx low
    DMX_TX_0;
    DMX_EN_1;
    TMR32_0_Init();
}

int DMX_GetChannel(uint8_t channel, uint8_t *R, uint8_t *G, uint8_t *B) {
    int resp  = -1;

    if (channel != 0) {
        uint8_t slot = ((channel - 1) * 3) + 1;
        *R = dmx_handler.data[slot++];
        *G = dmx_handler.data[slot++];
        *B = dmx_handler.data[slot++];
        resp = slot;
    }

    return (resp);
}


int DMX_GetChannelRGB(uint8_t channel, tp_dmx_rgb *rgb) {
    int resp  = -1;

    if (channel != 0) {
        uint8_t slot = ((channel - 1) * 3) + 1;
        rgb->R = dmx_handler.data[slot++];
        rgb->G = dmx_handler.data[slot++];
        rgb->B = dmx_handler.data[slot++];
        resp = slot;
    }

    return (resp);
}


void DMX_SetChannel(uint8_t channel, uint8_t R, uint8_t G, uint8_t B) {
    // Skip any zero channel!
    if (channel != 0) {
        uint8_t slot = ((channel - 1) * 3) + 1;

        dmx_handler.wanted_data[slot++] = R;
        dmx_handler.wanted_data[slot++] = G;
        dmx_handler.wanted_data[slot++] = B;
    }
}

void DMX_SetChannelRGB(uint8_t channel, tp_dmx_rgb *rgb) {
    // Skip any zero channel!
    if (channel != 0) {
        uint8_t slot = ((channel - 1) * 3) + 1;

        dmx_handler.wanted_data[slot++] = rgb->R;
        dmx_handler.wanted_data[slot++] = rgb->G;
        dmx_handler.wanted_data[slot++] = rgb->B;
    }
}

int DMX_GetRaw(uint8_t * raw_data)
{
    memcpy(raw_data, dmx_handler.data, DMX_SLOT_NUM);
    return(DMX_SLOT_NUM);
}


void DMX_SetRaw(uint8_t * raw_data, uint16_t len)
{
    memcpy(&dmx_handler.data[1], raw_data, len);
    //DEBUG(("DMX: %d,%d,%d\r\n", dmx_handler.data[0], dmx_handler.data[1], dmx_handler.data[2]));
}

void DMX_UpdateChannelRGB(void) {
    uint16_t i;
    if (!glb.conf.dmx_rate) {    // if rate is 0 then change color at once
        for (i = 0; i < DMX_SLOT_NUM; i++) dmx_handler.data[i] = dmx_handler.wanted_data[i];
    } else {  // if speed rate is other than 0 then change color gradually
        for (i = 0; i < DMX_SLOT_NUM; i++) {
            if (dmx_handler.data[i] != dmx_handler.wanted_data[i]) {
                if (dmx_handler.data[i] > dmx_handler.wanted_data[i]) {
                    dmx_handler.data[i] = (glb.conf.dmx_rate > (dmx_handler.data[i] - dmx_handler.wanted_data[i])) ?
                        dmx_handler.wanted_data[i] : dmx_handler.data[i] - glb.conf.dmx_rate;
                } else {
                    dmx_handler.data[i] = ((glb.conf.dmx_rate + dmx_handler.data[i]) > dmx_handler.wanted_data[i]) ?
                        dmx_handler.wanted_data[i] : dmx_handler.data[i] + glb.conf.dmx_rate;
                }
            }
        }   //:~ for each slot
    } //:~ rated
}


/**
 * @brief Change DMX state
 * 
 * @author jaco (6/1/2014)
 * 
 * @param state 
 */
__inline void DMX_Change_State(uint8_t state) {
    dmx_handler.state = state;
}


void DMX_Handler(void) {
    if (dmx_handler.state == DMX_STATE_DATA) {
        if (glb.dmx_mode == DMX_MODE_USER) DMX_UpdateChannelRGB();  // use dmx_rate only on user mode
        dmx_handler.data[0] = 0;    // /update DMX start byte to 0
        UART_SendBufferLen(dmx_handler.data, DMX_SLOT_NUM+1);   // Send data
        DMX_Change_State(DMX_STATE_WAIT_DATA);
    } else if (dmx_handler.state == DMX_STATE_WAIT_DATA) {
        // wait for data to be sent
        if (glb_uart_buffer.tx_length == 0) {
            // Repeat DMX data
            DMX_Change_State(DMX_STATE_MBB);
        }
    }
}

/**
 * @brief 32bit Timer interrupt routine
 * 
 * @author jaco (6/1/2014)
 * 
 * @return __irq void 
 */
__irq void TIMER32_0_IRQHandler(void) {
    if (LPC_TMR32B0->IR & 0x01) {
        LPC_TMR32B0->IR = 1;                /* clear interrupt flag */
        if (dmx_handler.state == DMX_STATE_ENABLE_485) {
            DMX_EN_1;
            DMX_Change_State(DMX_STATE_MBB);
            TM32_0_START(DMX_STATE_MBB);
        } else if (dmx_handler.state == DMX_STATE_MBB) {
            LPC_IOCON->PIO1_6  =  0;            // P1.6 is I/O
            LPC_IOCON->PIO1_7  =  0;            // P1.7 is I/O
            DMX_TX_1;   //PIN_SET(1,7,1); // Tx high and wait for
            DMX_Change_State(DMX_STATE_BREAK);
            TM32_0_START(DMX_TIMEOUT_MBB);
        } else if (dmx_handler.state == DMX_STATE_BREAK) {
            DMX_TX_0;   //PIN_SET(1,7,0);
            DMX_Change_State(DMX_STATE_MAB);
            TM32_0_START(DMX_TIMEOUT_BREAK);
        } else if (dmx_handler.state == DMX_STATE_MAB) {
            DMX_TX_0;   //PIN_SET(1,7,1);
            DMX_Change_State(DMX_STATE_READY);
            TM32_0_START(DMX_TIMEOUT_MAB);
        } else if (dmx_handler.state == DMX_STATE_READY) {
            DMX_TX_0;   //PIN_SET(1,7,0);
            LPC_IOCON->PIO1_6  =  (1UL <<  0);            // P1.6 is RxD
            LPC_IOCON->PIO1_7  =  (1UL <<  0);            // P1.7 is TxD
            DMX_Change_State(DMX_STATE_DATA);
        }
    }
}


/**
 * @brief Initialize 32bit timer
 * 
 * @author jaco (6/1/2014)
 */
void TMR32_0_Init(void) {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);    // Enable Tmr32_0
    DMX_TX_0;
    LPC_TMR32B0->PR = 1;
    LPC_TMR32B0->MCR = 3;                   // Interrupt and Reset on MR0
    NVIC_SetPriority(TIMER_32_0_IRQn, 1);   // Set interrupt Priority
    NVIC_EnableIRQ(TIMER_32_0_IRQn);

    DMX_Change_State(DMX_STATE_MBB);        // Set default state
    TM32_0_START(DMX_TIMEOUT_MBB * 2);
}



