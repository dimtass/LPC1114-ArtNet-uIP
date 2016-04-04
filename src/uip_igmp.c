#include "common.h"
#include "uip_igmp.h"
#include "uip.h"

#define DEBUG(X)   TRACEL(TRACE_LEVEL_UDP, X)

#define IGMP_MEMBERSHIP_QUERY       0x11 // Membership Query
#define IGMPv3_MEMBERSHIP_REPORT    0x22 //IGMP Ver. 3 Membership Report
#define IGMP_LEAVE_GROUP            0x17 //Leave Group

static struct igmp_connection_state s;

__packed
typedef struct {
    uint8_t type;
    uint8_t     max_resp_code;
    uint16_t    chksum;
    uint16_t    group_addr[2];
    uint8_t     options;
    uint8_t     qqic;   // Querier's query interval code
    uint16_t    num_of_src;
    uint16_t    src_addr[2];
} tp_igmp_query_h;

#define IGMP_Q ((tp_igmp_query_h *) uip_appdata)
#define IGMPBUF ((struct uip_igmpip_hdr *) &uip_buf[UIP_LLH_LEN])

void igmp_connection_init(void) 
{
    uip_ipaddr_t addr;

    // IGMPv3 multicast UDP listener
    // This is the port is used for the Internet Group Management Protocol
    //
    uip_ipaddr(addr, 224, 0, 0, 22);
    s.conn = uip_udp_new(&addr, HTONS(0));
    if (s.conn != NULL) {
        DEBUG(("Binding to UDP:\t%d.%d.%d.%d\r\n", 
               addr[0] & 0xFF, addr[0] >> 8,
               addr[1] & 0xFF, addr[1] >> 8));
        uip_udp_bind(s.conn, HTONS(0));
    }
}





void igmp_connection_appcall(void)
{
    s.conn = uip_udp_conn;
    if (uip_len) {
        DEBUG(("IGMPv3 data type: 0x%02x\r\n", IGMPBUF->type));
        if (IGMPBUF->type == IGMPv3_MEMBERSHIP_REPORT) {
            uip_ipaddr_t universeaddr;
            uip_ipaddr(universeaddr, 239, 255, 0, glb.conf.dmx_universe);

//          DEBUG(("gr:0x%04X%04X, uv:0x%04X%04X\r\n",
//                 IGMPBUF->groupaddr[0], IGMPBUF->groupaddr[1],
//                 universeaddr[0], universeaddr[1]
//                 ));

            if ( uip_ipaddr_cmp(IGMPBUF->groupaddr, universeaddr) ) {
                DEBUG(("Checked!\r\n"));
                // Respond to report with a query
                IGMP_Q->type = IGMP_MEMBERSHIP_QUERY;
                IGMP_Q->max_resp_code = 100;
                IGMP_Q->chksum = 0;
                uip_ipaddr(IGMP_Q->group_addr, 239, 255, 0, glb.conf.dmx_universe);
                IGMP_Q->options = 0;
                IGMP_Q->qqic = 0;
                IGMP_Q->num_of_src = 1;
                uip_ipaddr_copy(IGMP_Q->src_addr, glb.conf.uip_ipaddr);

                uip_send(uip_appdata, sizeof(tp_igmp_query_h));

            }
        }
        else if (IGMPBUF->type == IGMP_LEAVE_GROUP) {

        }
    }
}
