#include "common.h"
#include "udp_connection.h"
#include "uip.h"
#include "artnet.h"
#include "sACN.h"
#include "connection.h"
#include <stdio.h>
#include <string.h>

#define DEBUG(X)   TRACEL(TRACE_LEVEL_UDP, X)

#define DISCOVERY_UDP_PORT 7171
#define DISC_PREAMBLE       0xAAAA
#define DISC_HEADER ((struct tp_udp_disc_header *) data)
#define DISC_SEARCH_REPLY ((struct tp_udp_disc_search_resp *) data)
#define DISC_UPDATE_REQ ((struct tp_udp_disc_update_req *) data)

typedef enum {
    DISC_OPCODE_SEARCH_REQ = 0x4000,
    DISC_OPCODE_SEARCH_RESP,
    DISC_OPCODE_UPDATE_REQ,
    DISC_OPCODE_UPDATE_RESP
} en_disc_opcode;

__packed
struct tp_udp_disc_header {
    uint16_t    preamble;
    uint16_t    opcode;
    uint16_t    length;
};

__packed
struct tp_udp_disc_search_resp {
    uint16_t    preamble;
    uint16_t    opcode;
    uint16_t    length;
    uint8_t     version;
    uint8_t     mac[6];
    uint8_t     ipaddr[4];
    uint8_t     gateway[4];
    uint8_t     subnet[4];
    uint16_t    tcp_port;
};

#define DISC_OPCODE_UPDATE_REQ_VERSION  1
__packed
struct tp_udp_disc_update_req {
    /*version 1*/
    uint16_t    preamble;
    uint16_t    opcode;
    uint16_t    length;
    uint8_t     version;
    uint8_t     target_mac[6];
    uint8_t     ipaddr[4];
    uint8_t     gateway[4];
    uint8_t     subnet[4];
};


static struct udp_connection_state s;


/**
 * Discovery protocol 
 * This protocol is used to discover this device and change it's 
 * settings. 
 * 
 * @author jaco (24/11/2014)
 * 
 * @param data The incoming UDP data
 * @param len The incoming data length
 * 
 * @return uint16_t The length if the response (The response is 
 *         alredy in the uip_appdata buffer
 */
uint16_t udp_discovery_handler(uint8_t * data, uint16_t len)
{
    uint16_t retlen = 0;
    uint8_t *p;
    uint8_t i;

    if (DISC_HEADER->preamble == DISC_PREAMBLE) {   // check that is a valid packet
        DEBUG(("DISC opcode: %04X\r\n", DISC_HEADER->opcode));

        // Check packet opcode
        //
        if (DISC_HEADER->opcode == DISC_OPCODE_SEARCH_REQ) {

            // Discovery request
            // Respond to this with the following data
            DISC_SEARCH_REPLY->preamble = DISC_PREAMBLE;
            DISC_SEARCH_REPLY->opcode = DISC_OPCODE_SEARCH_RESP;
            DISC_SEARCH_REPLY->length = sizeof(struct tp_udp_disc_search_resp);
            DISC_SEARCH_REPLY->version = FW_VERSION;
            memcpy(DISC_SEARCH_REPLY->mac, glb.conf.uip_ethaddr.addr, 6);
            p = (uint8_t*) glb.conf.uip_ipaddr;
            for (i=0; i<4; i++) DISC_SEARCH_REPLY->ipaddr[i] = p[i];

            retlen = sizeof(struct tp_udp_disc_search_resp);
        }
        else if (DISC_HEADER->opcode == DISC_OPCODE_UPDATE_REQ) {
            // Remote update req.
            // Check if target MAC is ours and if it is then update the settings
            //
            if (memcmp(DISC_UPDATE_REQ->target_mac, glb.conf.uip_ethaddr.addr, 6) == 0) {
                
                init_ip();
//              uip_close();    // finilize uip
//
//              // Set default ip address
//              uip_ipaddr(glb.conf.uip_ipaddr, DISC_UPDATE_REQ->ipaddr[0], DISC_UPDATE_REQ->ipaddr[1],
//                         DISC_UPDATE_REQ->ipaddr[2], DISC_UPDATE_REQ->ipaddr[3]);
//              uip_sethostaddr(glb.conf.uip_ipaddr);
//
//              // Set default gateway
//              uip_ipaddr(glb.conf.uip_gateway, DISC_UPDATE_REQ->gateway[0], DISC_UPDATE_REQ->gateway[1],
//                         DISC_UPDATE_REQ->gateway[2], DISC_UPDATE_REQ->gateway[3]);
//              uip_setdraddr(glb.conf.uip_gateway);
//
//              // Set default subnet mask
//              uip_ipaddr(glb.conf.uip_subnet, DISC_UPDATE_REQ->subnet[0], DISC_UPDATE_REQ->subnet[1],
//                         DISC_UPDATE_REQ->subnet[2], DISC_UPDATE_REQ->subnet[3]);
//              uip_setnetmask(glb.conf.uip_subnet);
//
//              uip_init();             // Initialize uip
//              connection_init();      // re-init TCP
//              udp_connection_init();  // re-init UDP

                DEBUG(("ip: %d.%d.%d.%d\r\n", DISC_UPDATE_REQ->ipaddr[0], DISC_UPDATE_REQ->ipaddr[1],
                       DISC_UPDATE_REQ->ipaddr[2], DISC_UPDATE_REQ->ipaddr[3]));

                DEBUG(("gw: %d.%d.%d.%d\r\n", DISC_UPDATE_REQ->gateway[0], DISC_UPDATE_REQ->gateway[1],
                       DISC_UPDATE_REQ->gateway[2], DISC_UPDATE_REQ->gateway[3]));

                DEBUG(("gw: %d.%d.%d.%d\r\n", DISC_UPDATE_REQ->subnet[0], DISC_UPDATE_REQ->subnet[1],
                       DISC_UPDATE_REQ->subnet[2], DISC_UPDATE_REQ->subnet[3]));

                //FLASH_SAVE_AFTER(FLASH_SAVE_DFLT_DELAY);
                flash_save();
            }
        }
    }

    return(retlen);
}



void udp_connection_init(void) 
{
    uip_ipaddr_t addr;

    // Artnet broadcast listener
    // This UDP port is used for Art-Net communication ONLY
    //
    uip_ipaddr_copy(addr, glb.conf.uip_ipaddr); // copy local ip in order to get the subnet
    addr[1] &= 0xFF;        // clear last ip octet
    addr[1] |= 0xFF << 8;   // set it to broadcast
    //uip_ipaddr(addr, 192, 168, 1, 255);
    s.conn = uip_udp_new(&addr, HTONS(glb.conf.artnet_node.port));    // start listening to the broadcast ip x.x.x.255
    if (s.conn != NULL) {
        DEBUG(("Binding to UDP:\t%d.%d.%d.%d:%d\r\n", 
               addr[0] & 0xFF, addr[0] >> 8,
               addr[1] & 0xFF, addr[1] >> 8,
               ARTNET_UDP_PORT));
        uip_udp_bind(s.conn, HTONS(ARTNET_UDP_PORT));
    }

    // This UDP listener is used for the device discovery on the network.
    // The discovery protocol is explained above
    // 
    uip_ipaddr(addr, 224, 71, 71, 71);
    s.conn = uip_udp_new(&addr, HTONS(DISCOVERY_UDP_PORT));
    if (s.conn != NULL) {
        DEBUG(("Binding to UDP:\t%d.%d.%d.%d:%d\r\n", 
               addr[0] & 0xFF, addr[0] >> 8,
               addr[1] & 0xFF, addr[1] >> 8,
               DISCOVERY_UDP_PORT));
        uip_udp_bind(s.conn, HTONS(DISCOVERY_UDP_PORT));
    }

    // sACN multicast UDP listener
    // This is the port is used for the sACN protocol
    //
    uip_ipaddr(addr, 239, 255, 0, 0);
    addr[1] &= 0xFF;        // clear last ip octet
    addr[1] |= glb.conf.dmx_universe << 8;   // set it to universe number
    s.conn = uip_udp_new(&addr, HTONS(glb.conf.acn_port));
    if (s.conn != NULL) {
        DEBUG(("Binding to UDP:\t%d.%d.%d.%d:%d\r\n", 
               addr[0] & 0xFF, addr[0] >> 8,
               addr[1] & 0xFF, addr[1] >> 8,
               glb.conf.acn_port));
        uip_udp_bind(s.conn, HTONS(glb.conf.acn_port));
    }
}


void udp_connection_appcall(void) 
{
    // Get UDP connection state
    s.conn = uip_udp_conn;

    // At this position we know that the incoming data are from a valid ip and port.

    // Check if valid incoming UDP data
    if (uip_len) {
        DEBUG(("udp data: %d, %d\r\n", HTONS(s.conn->lport), HTONS(s.conn->rport)));

        if (s.conn->rport == HTONS(DISCOVERY_UDP_PORT)) {
            // Data to this port are from our own discovery protocol.
            // The parser is located in this file above
            uip_udp_send(udp_discovery_handler((uint8_t *)uip_appdata, uip_len));
        }
        else if (s.conn->lport == HTONS(ARTNET_UDP_PORT)) {
            // Parse artnet data and respond
            // The parser is located in artnet.c
            uip_udp_send(artnet_parser((uint8_t *)uip_appdata, uip_len));
        }
        else if (s.conn->lport == HTONS(glb.conf.acn_port)) {
            // Parse sACN data and respond
            // The parser is located in sACN.c
            uip_udp_send(acn_parser((uint8_t *)uip_appdata, uip_len));
        }
    }
}
