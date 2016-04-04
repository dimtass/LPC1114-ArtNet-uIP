/**
 * \addtogroup helloworld
 * @{
 */

/**
 * \file
 *         An example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/*
 * This is a short example of how to write uIP applications using
 * protosockets.
 */

/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "common.h"
#include "connection.h"
#include "uip.h"
#include "dmx.h"
#include <string.h>
#include <stdio.h>

#define DEBUG(X)    TRACEL(TRACE_LEVEL_TCP, X)

#define DMX_TCP_PREAMBLE    0xBBBB
#define DMX_HEADER  ((tp_dmx_tcp_header*) uip_appdata)
#define DMX_INFO    ((tp_dmx_tcp_info*) uip_appdata)
#define DMX_DATA    ((tp_dmx_tcp_channel*) uip_appdata)

typedef enum {
    DMX_TCP_RGB_INFO_REQ = 0x2900,
    DMX_TCP_RGB_INFO_RESP = 0x2901,
    DMX_TCP_RGB_CH_REQ = 0x2902,
    DMX_TCP_RGB_CH_SET = 0x2903,
    DMX_TCP_RAW_REQ = 0x2904,
    DMX_TCP_RAW_RESP = 0x2905,
    DMX_TCP_RAW_SET = 0x2906
} en_dmx_tcp_opcode;


#define DMX_TCP_VERSION 1

__packed
typedef struct {
    uint16_t    preamble;
    uint16_t    opcode;
} tp_dmx_tcp_header;

__packed
typedef struct {
    uint16_t    preamble;
    uint16_t    opcode;
    uint8_t     version;
    uint8_t     rate;
    uint8_t     supported_channels;
} tp_dmx_tcp_info;

__packed
typedef struct {
    uint16_t    preamble;
    uint16_t    opcode;
    uint8_t     channel;    // dmx channel to set
    uint8_t     R;
    uint8_t     G;
    uint8_t     B;
} tp_dmx_tcp_channel;


/**
 * 
 * 
 * @author jaco (17/11/2014)
 * 
 * @param data 
 * @param data_len 
 * 
 * @return uint16_t 
 */
void connection_handle_binary() {

    if (uip_newdata()) {
        uint16_t send_len = 0;

        if (DMX_HEADER->preamble == DMX_TCP_PREAMBLE) {
            DEBUG(("DMX_TCP->opcode: %04X\r\n", DMX_HEADER->opcode));
            if (DMX_INFO->opcode == DMX_TCP_RGB_INFO_REQ) {
                DMX_INFO->opcode = DMX_TCP_RGB_INFO_RESP;
                DMX_INFO->version = DMX_TCP_VERSION;
                DMX_INFO->rate = glb.conf.dmx_rate;
                DMX_INFO->supported_channels = glb.conf.dmx_slots / 3;
                send_len = sizeof(tp_dmx_tcp_info);
            }
            else if (DMX_INFO->opcode == DMX_TCP_RGB_CH_REQ) {
                DMX_GetChannelRGB(DMX_DATA->channel, (tp_dmx_rgb*) &DMX_DATA->R);
                //DEBUG(("Get ch,R,G,B: %d,%d,%d,%d\r\n", DMX_DATA->channel, rgb->R, rgb->G, rgb->B));
                send_len = sizeof(tp_dmx_tcp_channel);
            }
            else if (DMX_INFO->opcode == DMX_TCP_RGB_CH_SET) {
                //DEBUG(("Set ch,R,G,B: %d,%d,%d,%d\r\n", DMX_DATA->channel, DMX_DATA->R, DMX_DATA->G, DMX_DATA->B));
                DMX_SetChannelRGB(DMX_DATA->channel, (tp_dmx_rgb*) &DMX_DATA->R);
            }
            else if (DMX_INFO->opcode == DMX_TCP_RAW_REQ) {
                DMX_HEADER->opcode = DMX_TCP_RAW_RESP;
                //TODO: finish this command
            }
        }

        if (send_len) {
            uip_send(uip_appdata, send_len);
        }
    }

//  if (uip_closed() || uip_aborted() || uip_timedout()) {
//      // what to do here?
//  }
//
//  if (uip_acked()) {
//      // what to d here?
//  }
//
//  if (uip_rexmit() || uip_newdata() || uip_acked() ||
//      uip_connected() || uip_poll()) {
//      senddata();
//  }

}


/**
 * 
 * 
 * @author jaco (17/11/2014)
 * 
 * @return uint16_t 
 */
uint16_t connection_handle_ascii(struct connection_state *s) {
    tp_dmx_rgb rgb;
    char str[56];
    static uint16_t i;  // i is static because of recursive run
    uint16_t channel = atoi2(&s->inputbuffer[4]);   // get which channel to send
    PSOCK_BEGIN(&s->p);

    while (1) {
        PSOCK_READTO(&s->p, '\n');
        {
            // Get RGB values
            if (!strncmp(s->inputbuffer, "RGB?", 4)) {

                PSOCK_SEND_STR(&s->p, "RGB=");
                if (channel == 0) { // If 0 then send all channels
                    for (i = 1; i <= DMX_CHANNELS_NUM; i++) {
                        DMX_GetChannelRGB(i, &rgb); // Get DMX values
                        sprintf(str, "%d:(%d,%d,%d),", i, rgb.R, rgb.G, rgb.B);
                        PSOCK_SEND_STR(&s->p, str);
                    }
                } else {  // Send only requested channel
                    DMX_GetChannelRGB(channel, &rgb); // Get DMX values
                    sprintf(str, "%d:(%d,%d,%d)", channel, rgb.R, rgb.G, rgb.B);
                    PSOCK_SEND_STR(&s->p, str);
                }
                PSOCK_SEND_STR(&s->p, "\n");
            }
            // Set an RGB channel to the given R,G,B values
            // Format:
            // RGB=[ch],[R],[G],[B]
            // RGB=2,150,100,50
            // means, set RGB channel 2 to R=150, G=100, B=50
            else if (!strncmp(s->inputbuffer, "RGB=", 4)) {
                // TRACE command
                char *pch;
                uint8_t data[4];
                uint8_t index = 0;

                pch = strtok(&s->inputbuffer[4], ",");
                while (pch != NULL) {
                    data[index++] = atoi2(pch);
                    pch = strtok(NULL, ",");
                }
                DMX_SetChannel(data[0], data[1], data[2], data[3]);
            }
            // Set rate 0: no delay, >1: rate from slow to fast
            else if (!strncmp(s->inputbuffer, "RATE", 4)) {
                if (s->inputbuffer[4] == '?') {  // Print rate
                    sprintf(str, "RATE=%d\n", glb.conf.dmx_rate);
                    PSOCK_SEND_STR(&s->p, str);
                } else if (s->inputbuffer[4] == '=') {    // Set rate
                    glb.conf.dmx_rate = atoi2(&s->inputbuffer[5]);
                    sprintf(str, "RATE set to: %d\n", glb.conf.dmx_rate);
                    PSOCK_SEND_STR(&s->p, str);
                }
            } else if (!strncmp(s->inputbuffer, "SAVE", 4)) {
                //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                flash_save();
            }
        }
    }

    PSOCK_SEND_STR(&s->p, "Done");
    PSOCK_CLOSE(&s->p);

    PSOCK_END(&s->p);
}


/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void connection_init(void) {
    /* We start to listen for connections on TCP port 1000. */
    uip_listen(HTONS(7171));
}
/*---------------------------------------------------------------------------*/
/*
 * In hello-world.h we have defined the UIP_APPCALL macro to
 * hello_world_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void connection_appcall(void) {
    /*
     * The uip_conn structure has a field called "appstate" that holds
     * the application state of the connection. We make a pointer to
     * this to access it easier.
     */
    struct connection_state *s = &(uip_conn->appstate);

    /*
     * If a new connection was just established, we should initialize
     * the protosocket in our applications' state structure.
     */
    if (uip_connected() && (glb.conf.dmx_tcp_mode == DMX_TCP_MODE_ASCII)) {
        PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
        glb.dmx_mode = DMX_MODE_USER;   // go to user TCP dmx mode
    }

    // handle ASCII mode connections
    if (glb.conf.dmx_tcp_mode == DMX_TCP_MODE_ASCII) {
        connection_handle_ascii(s);
    }
    // handle HEX/binary mode connections
    else {
        connection_handle_binary();
    }
}
