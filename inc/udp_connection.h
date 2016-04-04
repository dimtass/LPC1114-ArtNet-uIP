
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

#ifndef __UDP_CONNECTION_H__
#define __UDP_CONNECTION_H__
#pragma once


/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */

struct udp_connection_state {
  char state;
  struct uip_udp_conn *conn;
  uint8_t *  data;
};

typedef struct udp_connection_state uip_udp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void udp_connection_appcall(void);
#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL udp_connection_appcall
#endif /* UIP_APPCALL */

void udp_connection_init(void);

#endif /* __HELLO_WORLD_H__ */
/** @} */
/** @} */
