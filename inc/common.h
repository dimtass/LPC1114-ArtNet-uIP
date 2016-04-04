#ifndef COMMON_H__
#define COMMON_H__
#pragma once

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

#include <stdio.h>
#include "LPC11xx.h"
#include "uip.h"
#include "artnet.h"

#define FW_VERSION      1
#define FLASH_VERSION   1

typedef enum {
    TRACE_LEVEL_GENERIC = 1,
    TRACE_LEVEL_DM9K = 2,
    TRACE_LEVEL_UIP = 4,
    TRACE_LEVEL_RGB = 8,
    TRACE_LEVEL_DMX = 16,
    TRACE_LEVEL_ARTNET = 32,
    TRACE_LEVEL_UDP = 64,
    TRACE_LEVEL_TCP = 128,
    TRACE_LEVEL_ACN = 256
} en_TRACE_LEVELS;

#define TRACE_ENABLED

#ifdef TRACE_ENABLED
#define TRACEL(L, X) if (glb.trace_levels & L) printf X     // Trace with a given level
#else
#define TRACEL(X, L)    // Trace with a given level
#endif

#define PIN_SD0     4
#define PIN_SD1     5
#define PIN_SD2     6
#define PIN_SD3     7
#define PIN_SD4     8
#define PIN_SD5     9
#define PIN_SD6     10
#define PIN_SD7     11

#define PIN_IOW     2
#define PIN_IOR     3
#define PIN_PWRST   8
#define PIN_CMD     9
#define PIN_DEBUG   0
#define PIN_CFG     3
#define PIN_485_EN  1

#define DATA_PORT0_INP	(1<<PIN_IOR)|(1<<PIN_IOW) //0x000C  // SD7-SD2: inputs, IOR & IOW: outputs
#define DATA_PORT0_OUT	(1<<PIN_SD7)|(1<<PIN_SD6)|(1<<PIN_SD5)|(1<<PIN_SD4)|(1<<PIN_SD3)|(1<<PIN_SD2)|(1<<PIN_IOR)|(1<<PIN_IOW) //0x0FCC  // SD7-SD2: outputs, IOR & IOW: outputs
#define DATA_PORT3_INP	0       // SD1-SD0: inputs
#define DATA_PORT3_OUT	(1<<PIN_SD1)|(1<<PIN_SD0)   //0x0030  // SD1-SD0: outputs
#define PORT0_DATA_MASK 0x00FC  // mask the 6 MSb for port 0
#define PORT3_DATA_MASK 0x0003  // mask the 2 LSb for port 3
#define PORT0_DATA_POS  6       // data starts from P0.6
#define PORT3_DATA_POS  4       // data starts from P3.4


#define DEBUG_PIN_HIGH  LPC_GPIO2->DATA |= (1<<PIN_DEBUG)
#define DEBUG_PIN_LOW   LPC_GPIO2->DATA &= ~(1<<PIN_DEBUG)


typedef struct {
    uint32_t    header;                 // must always be 0ABCDEF0
    uint32_t    version;                // flash version
    
    // uip parameters
    uip_ipaddr_t uip_ipaddr;            // local static ip
    uip_ipaddr_t uip_gateway;           // gateway
    uip_ipaddr_t uip_subnet;            // subnet mask
    struct uip_eth_addr uip_ethaddr;    // MAC address

    // artnet params
    tp_artnet_node  artnet_node;

    // sACN params
    uint16_t        acn_port;

    // dmx params
    uint16_t    dmx_slots;          // number of supported slots
    uint8_t     dmx_rate;           //0: dmx changes color immediately, >1: changes with given rate
    uint8_t     dmx_tcp_mode;       //0: ASCII mode, 1: binary mode (see: connection.c)
    uint16_t    dmx_universe;

} tp_conf;


#define FLASH_SAVE_DFLT_DELAY   2
typedef struct {
    uint8_t     debug;              //0: use 485, 1: use rs232
    uint32_t    timestamp;
    uint16_t    ticks;
    uint16_t    dm9k_tick_count;
    uint32_t    trace_levels;
    uint8_t     reinit;
    uint8_t     dmx_mode;           // 0: User TCP mode, >=1: Art-Net

    tp_conf     conf;

} tp_glb;

extern tp_glb glb;
extern int atoi2(const char *s);
extern uint8_t flash_save(void);
extern void flash_restore_defaults(void);
extern void init_ip(void);

#endif //COMMON_H__
