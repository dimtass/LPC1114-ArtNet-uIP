/**
 * Functions for Art-Net lighting protocol 
 *  
 * author: dimtass 
 */


#ifndef __ARTNET_H__
#define __ARTNET_H__
#pragma once


#define ARTNET_MAX_PORTS            4
#define ARTNET_SHORT_NAME_LENGTH    18
#define ARTNET_LONG_NAME_LENGTH     64
#define ARTNET_REPORT_LENGTH        64
#define ARTNET_DMX_LENGTH           512
#define ARTNET_MAC_SIZE             6
#define ARTNET_UDP_PORT 6454

__packed
typedef struct {
    uint16_t    address;
    uint16_t    port;
    uint8_t     type;       // see: artnet_node_type_en
    uint8_t     mode;       // see: node_status_en
    uint8_t     subnet_net_ctl;
    uint16_t    send_apr_on_change;
    uint16_t    ar_count;
    uint16_t    verbose;
    uint8_t     short_name[ARTNET_SHORT_NAME_LENGTH];
    uint8_t     long_name[ARTNET_LONG_NAME_LENGTH];
    uint8_t     report[ARTNET_REPORT_LENGTH];
    uint8_t     port_types[ARTNET_MAX_PORTS];
    uint8_t     port_address[ARTNET_MAX_PORTS]; // address from 1-4 for each port
    uint16_t    oem;
    uint16_t    esta_code;
    uint16_t    bcast_limit; // the number of nodes after which we change to bcast
    uint8_t     report_code;  // see: artnet_node_report_code_en
} tp_artnet_node;

void        artnet_defaults(void);
void        artnet_init(tp_artnet_node * node_info);
uint16_t    artnet_parser(uint8_t * data, uint16_t datalen);

#endif //__ARTNET_H__
