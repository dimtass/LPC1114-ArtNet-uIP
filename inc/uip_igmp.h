#ifndef __UIP_IGMP_H__
#define __UIP_IGMP_H__
#pragma once

/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */

struct igmp_connection_state {
  char state;
  struct uip_udp_conn *conn;
  uint8_t *  data;
};

typedef struct igmp_connection_state uip_igmp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void igmp_connection_appcall(void);

#define UIP_IGMP_APPCALL igmp_connection_appcall

void igmp_connection_init(void);



#endif

