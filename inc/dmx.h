#ifndef DMX_H__
#define DMX_H__
#pragma once

#define DMX_CHANNELS_NUM    170
#define DMX_SLOT_NUM        512

typedef enum {
    DMX_STATE_START,
    DMX_STATE_ENABLE_485,
    DMX_STATE_MBB,      // MARK before BREAK (MBB)  < 1sec
    DMX_STATE_BREAK,    // SPACE for BREAK          = 176 usec
    DMX_STATE_MAB,      // MARK after BREAK (MAB)   = 12 usec
    DMX_STATE_READY,
    DMX_STATE_DATA,
    DMX_STATE_WAIT_DATA,
    DMX_STATE_DISABLE_485,
    DMX_STATE_RDM_RX,
    DMX_STATE_RDM_TX
} en_dmx_state;


typedef enum { 
    DMX_MODE_USER,
    DMX_MODE_ARTNET,
    DMX_MODE_RDM_TX,
    DMX_MODE_RDM_RX
} dmx_mode_en;


typedef struct {
    uint8_t         state;  //en_dmx_state
    uint8_t         data[DMX_SLOT_NUM+1];  // Start slot 0 + 512 DMX values
    uint8_t         wanted_data[DMX_SLOT_NUM+1];  // 512 DMX values
    uint8_t         rate;   // rate speed from one value to another
} tp_dmx_handler;

__packed
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
}
tp_dmx_rgb;

extern tp_dmx_handler dmx_handler;


/**
 * @brief Initialize the DMX 485 channel and timers
 * 
 * @author jaco (6/1/2014)
 */
void DMX_Init(void);


/**
 * @brief DMX handler. This must be called in main loop
 * 
 * @author jaco (6/1/2014)
 */
void DMX_Handler(void);


/**
 * @brief Get channel's RGB value
 * 
 * @author jaco (6/1/2014)
 * 
 * @param channel 
 * @param R Red value
 * @param G Green value
 * @param B Blue value
 * 
 * @return int -1: on error, >0 else
 */
int DMX_GetChannel(uint8_t channel, uint8_t *R, uint8_t *G, uint8_t *B);
int DMX_GetChannelRGB(uint8_t channel, tp_dmx_rgb *rgb);


/**
 * @brief Set DMX512 channel colors
 * 
 * @author jaco (6/1/2014)
 * 
 * @param channel 
 * @param R Red value
 * @param G Green Value
 * @param B Blue Value
 */
void DMX_SetChannel(uint8_t channel, uint8_t R, uint8_t G, uint8_t B);
void DMX_SetChannelRGB(uint8_t channel, tp_dmx_rgb *rgb);


/**
 * Get/Set raw DMX data
 * 
 * @author jaco (17/11/2014)
 * 
 * @param raw_data 
 * 
 * @return int 
 */
int DMX_GetRaw(uint8_t * raw_data);
void DMX_SetRaw(uint8_t * raw_data, uint16_t len);

#endif
