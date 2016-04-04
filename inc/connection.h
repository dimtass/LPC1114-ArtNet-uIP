/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup helloworld Hello, world
 * @{
 *
 * A small example showing how to write applications with
 * \ref psock "protosockets".
 */

/**
 * \file
 *         Header file for an example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#ifndef __CONNECTION_H__
#define __CONNECTION_H__
#pragma once

/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */
#include "uipopt.h"
#include "psock.h"

typedef enum {
    DMX_TCP_MODE_ASCII,
    DMX_TCP_MODE_BINARY
}en_dmx_tcp_mode;

/* Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
typedef struct connection_state {
    struct psock p;
    char inputbuffer[100];
    char name[40];
} uip_tcp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void connection_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL connection_appcall
#endif /* UIP_APPCALL */

void connection_init(void);

#endif /* __HELLO_WORLD_H__ */
/** @} */
/** @} */
