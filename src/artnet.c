#include "common.h"
#include "artnet.h"
#include "dmx.h"
#include <stdio.h>
#include <string.h>



#define DEBUG(X)   TRACEL(TRACE_LEVEL_ARTNET, X)
#define ARTNET_HEADER ((struct artnet_header_tp *) data)
#define ARTNET_POLL_REPLY ((struct artnet_poll_reply_tp *) data)
#define ARTNET_IPPROG_MSG ((struct artnet_ip_prog_msg_tp *) data)
#define ARTNET_IPPROG_REPLY ((struct artnet_ip_prog_reply_tp *) data)
#define ARTNET_DMX ((struct artnet_dmx_tp *) data)
#define ARTNET_ADDRESS ((struct artnet_adress_tp *) data)



typedef enum {
  ARTNET_SRV,      /**< An ArtNet server (transmitts DMX data) */
  ARTNET_NODE,    /**< An ArtNet node   (dmx reciever) */
  ARTNET_MSRV,    /**< A Media Server */
  ARTNET_ROUTE,    /**< No Effect currently */
  ARTNET_BACKUP,    /**< No Effect currently */
  ARTNET_RAW      /**< Raw Node - used for diagnostics */
} artnet_node_type_en;

// the status of the node
typedef enum {
  ARTNET_OFF,
  ARTNET_STANDBY,
  ARTNET_ON
} node_status_en;

typedef enum {
  ARTNET_RCDEBUG,
  ARTNET_RCPOWEROK,
  ARTNET_RCPOWERFAIL,
  ARTNET_RCSOCKETWR1,
  ARTNET_RCPARSEFAIL,
  ARTNET_RCUDPFAIL,
  ARTNET_RCSHNAMEOK,
  ARTNET_RCLONAMEOK,
  ARTNET_RCDMXERROR,
  ARTNET_RCDMXUDPFULL,
  ARTNET_RCDMXRXFULL,
  ARTNET_RCSWITCHERR,
  ARTNET_RCCONFIGERR,
  ARTNET_RCDMXSHORT,
  ARTNET_RCFIRMWAREFAIL,
  ARTNET_RCUSERFAIL
} artnet_node_report_code_en;

typedef enum {
  STNODE = 0x00,
  STSERVER = 0x01,
  STMEDIA = 0x02,
  STROUTE = 0x03,
  STBACKUP = 0x04,
  STCONFIG = 0x05
} artnet_node_style_code_en;


typedef enum {
    ARTNET_OPCODE_POLL = 0x2000,
    ARTNET_OPCODE_REPLY = 0x2100,
    ARTNET_OPCODE_DMX = 0x5000,
    ARTNET_OPCODE_ADDRESS = 0x6000,
    ARTNET_OPCODE_INPUT = 0x7000,
    ARTNET_OPCODE_TODREQUEST = 0x8000,
    ARTNET_OPCODE_TODDATA = 0x8100,
    ARTNET_OPCODE_TODCONTROL = 0x8200,
    ARTNET_OPCODE_RDM = 0x8300,
    ARTNET_OPCODE_VIDEOSTEUP = 0xa010,
    ARTNET_OPCODE_VIDEOPALETTE = 0xa020,
    ARTNET_OPCODE_VIDEODATA = 0xa040,
    ARTNET_OPCODE_MACMASTER = 0xf000,
    ARTNET_OPCODE_MACSLAVE = 0xf100,
    ARTNET_OPCODE_FIRMWAREMASTER = 0xf200,
    ARTNET_OPCODE_FIRMWAREREPLY = 0xf300,
    ARTNET_OPCODE_IPPROG = 0xf800,
    ARTNET_OPCODE_IPREPLY = 0xf900,
    ARTNET_OPCODE_MEDIA = 0x9000,
    ARTNET_OPCODE_MEDIAPATCH = 0x9200,
    ARTNET_OPCODE_MEDIACONTROLREPLY = 0x9300
} artnet_packet_en;

__packed
typedef struct {
    uint8_t ubea_present:1;
    uint8_t dual_boot:1;
    uint8_t nu:1;
    uint8_t papa:3;     // Port Address Programming Authority
    uint8_t led_state;  //0: unknown, 1: locate mode, 2: mute mode, 3: normal mode
} tp_artnet_status1;



__packed
struct artnet_header_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
};

__packed
struct artnet_header2_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
};


__packed
struct artnet_poll_msg_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
    uint8_t     talk_to_me;
    uint8_t     priority;
};

__packed
struct artnet_poll_reply_tp {
    uint8_t  id[8];
    uint16_t opCode;
    uint8_t  ip[4];
    uint16_t port;
    uint16_t ver;
    uint8_t  net;
    uint8_t  sub;
    uint16_t oem;
    uint8_t  ubea;
    uint8_t  status;
    uint16_t esta_code;
    uint8_t  shortname[ARTNET_SHORT_NAME_LENGTH];
    uint8_t  longname[ARTNET_LONG_NAME_LENGTH];
    uint8_t  nodereport[ARTNET_REPORT_LENGTH];
    uint16_t numbports;
    uint8_t  porttypes[ARTNET_MAX_PORTS];
    uint8_t  input_status[ARTNET_MAX_PORTS];
    uint8_t  output_status[ARTNET_MAX_PORTS];
    uint8_t  input_subswitch[ARTNET_MAX_PORTS];
    uint8_t  output_subswitch[ARTNET_MAX_PORTS];
    uint8_t  swvideo;
    uint8_t  swmacro;
    uint8_t  swremote;
    uint8_t  sp1;
    uint8_t  sp2;
    uint8_t  sp3;
    uint8_t  style;
    uint8_t  mac[ARTNET_MAC_SIZE];
    uint8_t  filler[29];
};

#define ARTNET_IPPROG_ENABLE    (1<<7)
#define ARTNET_IPPROG_DCHP      (1<<6)
#define ARTNET_IPPROG_DEF       (1<<3)
#define ARTNET_IPPROG_IP        (1<<2)
#define ARTNET_IPPROG_SUB       (1<<1)
#define ARTNET_IPPROG_PORT      (1<<0)
struct artnet_ip_prog_msg_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
    uint8_t     nu1[2];
    uint8_t     command;
    uint8_t     nu2;
    uint8_t     ip[4];
    uint8_t     subnet[4];
    uint16_t    port;
    uint8_t     nu3[8];
};

struct artnet_ip_prog_reply_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
    uint8_t     nu1[4];
    uint8_t     ip[4];  //MSB->LSB
    uint8_t     subnet[4];  //MSB-LSB
    uint16_t    port;
    uint8_t     status; // always 0
    uint8_t     nu2[7];
};

struct artnet_dmx_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
    uint8_t     seq;        // The sequence number is used to ensure that ArtDmx packets are used in the correct order. 
                            //When Art-Net is carried over a medium such as the Internet, 
                            //it is possible that ArtDmx packets will reach the receiver out of order. 
                            //This field is incremented in the range 0x01 to 0xff to allow the receiving node 
                            //to resequence packets. 
                            //The Sequence field is set to 0x00 to disable this feature.
    uint8_t     physical;   // The physical input port from which DMX512 data was input.
    uint8_t     sub_uni;    // The low byte of the 15 bit Port-Address to which this packet is destined.
    uint8_t     net;        // The top 7 bits of the 15 bit Port-Address to which this packet is destined.
    uint16_t    length;     // The length of the DMX512 data array. This value should be an even number in the range 2 – 512.
    uint8_t     data[];      // pointer to data
};


struct artnet_adress_tp {
    uint8_t     id[8];          // "Art-Net\0"
    uint16_t    opCode;
    uint8_t     prot_ver_hi;
    uint8_t     prot_ver_low;
    uint8_t     net;
    uint8_t     nu1;
    uint8_t     short_name[18];
    uint8_t     long_name[64];
    uint8_t     swin[4];
    uint8_t     swout[4];
    uint8_t     sub;
    uint8_t     swvideo;
    uint8_t     command;
};



void artnet_defaults(void)
{
    glb.conf.artnet_node.type = ARTNET_NODE;
    glb.conf.artnet_node.port = ARTNET_UDP_PORT;
    glb.conf.artnet_node.mode = ARTNET_ON;
    glb.conf.artnet_node.subnet_net_ctl = 0;
    glb.conf.artnet_node.send_apr_on_change = 0;
    glb.conf.artnet_node.ar_count = 0;
    glb.conf.artnet_node.verbose = 0;
    memset((char*)glb.conf.artnet_node.short_name, '\0', ARTNET_SHORT_NAME_LENGTH);
    sprintf((char*)glb.conf.artnet_node.short_name, "Jaco");
    memset((char*)glb.conf.artnet_node.long_name, '\0', ARTNET_LONG_NAME_LENGTH);
    sprintf((char*)glb.conf.artnet_node.long_name, "Jaco Art-Net Ethernet to DMX node");
    memset((char*)glb.conf.artnet_node.report, '\0', ARTNET_REPORT_LENGTH);
    sprintf((char*)glb.conf.artnet_node.report, "Input subnet: 1 Output subnet: 0");
    glb.conf.artnet_node.oem = 0xffff;
    glb.conf.artnet_node.esta_code = 0;
    glb.conf.artnet_node.bcast_limit = 1;
    glb.conf.artnet_node.report_code = ARTNET_RCPOWEROK;
    glb.conf.artnet_node.port_types[0] = 0x40;   // 1 DMX port output
    glb.conf.artnet_node.port_types[1] = glb.conf.artnet_node.port_types[2] =
    glb.conf.artnet_node.port_types[3] = 0;
    glb.conf.artnet_node.address = 0;
    glb.conf.artnet_node.port_address[0] = 1;
    glb.conf.artnet_node.port_address[1] = glb.conf.artnet_node.port_address[2] = 
        glb.conf.artnet_node.port_address[3] = 0;
}

void artnet_init(tp_artnet_node * node_info)
{
    DEBUG(("Art-Net initialized\r\n"));
    memcpy(&glb.conf.artnet_node, node_info, sizeof(tp_artnet_node));
}


uint16_t artnet_get_poll_reply(uint8_t * data)
{
    uint8_t *p;
    uint8_t i;

    //fill packet with necessary stuf

    memcpy(ARTNET_POLL_REPLY->id, "Art-Net\0", 8);  // preamble
    ARTNET_POLL_REPLY->opCode = ARTNET_OPCODE_REPLY;       // op code
                                                    //
    p = (uint8_t*) glb.conf.uip_ipaddr;                      // ip
    for (i=0; i<4; i++) ARTNET_POLL_REPLY->ip[i] = p[i];

    ARTNET_POLL_REPLY->port = ARTNET_UDP_PORT;

    ARTNET_POLL_REPLY->ver = 0;

    ARTNET_POLL_REPLY->net = (glb.conf.artnet_node.address >> 8) & 0x7F;
    ARTNET_POLL_REPLY->sub = (glb.conf.artnet_node.address >> 4) & 0x0F;

    ARTNET_POLL_REPLY->oem = glb.conf.artnet_node.oem;

    ARTNET_POLL_REPLY->ubea = 0;

    ARTNET_POLL_REPLY->status = 0xf0;

    ARTNET_POLL_REPLY->esta_code = glb.conf.artnet_node.esta_code;

    memcpy(ARTNET_POLL_REPLY->shortname, glb.conf.artnet_node.short_name, ARTNET_SHORT_NAME_LENGTH);

    memcpy(ARTNET_POLL_REPLY->longname, glb.conf.artnet_node.long_name, ARTNET_LONG_NAME_LENGTH);

    memcpy(ARTNET_POLL_REPLY->nodereport, glb.conf.artnet_node.report, ARTNET_REPORT_LENGTH);

    ARTNET_POLL_REPLY->numbports = HTONS(1);

    memcpy(ARTNET_POLL_REPLY->porttypes, glb.conf.artnet_node.port_types, 4);

    for (i=0; i<ARTNET_MAX_PORTS; i++) 
        ARTNET_POLL_REPLY->input_status[i] = ARTNET_POLL_REPLY->output_status[i] = 0;

    for (i=0; i<ARTNET_MAX_PORTS; i++)
         ARTNET_POLL_REPLY->input_subswitch[i] = 0;

    for (i=0; i<ARTNET_MAX_PORTS; i++)
        ARTNET_POLL_REPLY->output_subswitch[i] = glb.conf.artnet_node.port_address[i] & 0x07;

    ARTNET_POLL_REPLY->swvideo = 0;
    ARTNET_POLL_REPLY->swmacro = 0;
    ARTNET_POLL_REPLY->swremote = 0;

    ARTNET_POLL_REPLY->sp1 = ARTNET_POLL_REPLY->sp2 = ARTNET_POLL_REPLY->sp3 = 0;

    ARTNET_POLL_REPLY->style = STNODE;

    memcpy(ARTNET_POLL_REPLY->mac, glb.conf.uip_ethaddr.addr, 6);

    memset(ARTNET_POLL_REPLY->filler, 0, 32);

    return(sizeof(struct artnet_poll_reply_tp));
}


uint16_t artnet_handle_address(uint8_t * data)
{
//  uint8_t i;

    if (ARTNET_ADDRESS->net & 0x80) {
        glb.conf.artnet_node.address &= ~(0x7F << 8);
        glb.conf.artnet_node.address |= (ARTNET_ADDRESS->net & 0x7F) << 8; 
    }
    if (ARTNET_ADDRESS->sub & 0x80) {
        glb.conf.artnet_node.address &= ~(0x0F << 4);
        glb.conf.artnet_node.address |= (ARTNET_ADDRESS->sub & 0x0F) << 4; 
    }

//  for (i=0; i<ARTNET_MAX_PORTS; i++) {
//      glb.conf.artnet_node.port_address[i] = ARTNET_ADDRESS->swout[i];
//  }
    glb.conf.artnet_node.port_address[0] = ARTNET_ADDRESS->swout[0];

    return(0);
}



uint16_t artnet_handle_ipprog_reply(uint8_t * data)
{
    // need programming?
    if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_ENABLE) {
        // Reset to defaults?
        if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_DEF) {
            // Reset to defaults
            flash_restore_defaults();
            flash_save();
        }
        else if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_DCHP) {
            // Do not support this
        } 
        else {
            if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_IP) {
                uip_ipaddr(glb.conf.uip_ipaddr, ARTNET_IPPROG_MSG->ip[0], 
                           ARTNET_IPPROG_MSG->ip[1], 
                           ARTNET_IPPROG_MSG->ip[2], 
                           ARTNET_IPPROG_MSG->ip[3]);
                uip_sethostaddr(glb.conf.uip_ipaddr);
                glb.reinit = 1;
            }
            if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_SUB) {
                uip_ipaddr(glb.conf.uip_subnet, ARTNET_IPPROG_MSG->subnet[0],
                           ARTNET_IPPROG_MSG->subnet[1], 
                           ARTNET_IPPROG_MSG->subnet[2], 
                           ARTNET_IPPROG_MSG->subnet[3]);
                uip_setnetmask(glb.conf.uip_subnet);
                glb.reinit = 1;
            }
            if (ARTNET_IPPROG_MSG->command & ARTNET_IPPROG_PORT) {
                glb.conf.artnet_node.port = HTONS(ARTNET_IPPROG_MSG->port);
                glb.reinit = 1;
            }
        }
    }

    memcpy(ARTNET_IPPROG_REPLY->id, "Art-Net\0", 8);  // preamble
    ARTNET_IPPROG_REPLY->opCode = ARTNET_OPCODE_IPREPLY;       // op code
    ARTNET_IPPROG_REPLY->ip[0] = glb.conf.uip_ipaddr[0] & 0xFF;
    ARTNET_IPPROG_REPLY->ip[1] = glb.conf.uip_ipaddr[0] >> 8;
    ARTNET_IPPROG_REPLY->ip[2] = glb.conf.uip_ipaddr[1] & 0xFF;
    ARTNET_IPPROG_REPLY->ip[3] = glb.conf.uip_ipaddr[1] >> 8;

    ARTNET_IPPROG_REPLY->subnet[0] = glb.conf.uip_subnet[0] & 0xFF;
    ARTNET_IPPROG_REPLY->subnet[1] = glb.conf.uip_subnet[0] >> 8;
    ARTNET_IPPROG_REPLY->subnet[2] = glb.conf.uip_subnet[1] & 0xFF;
    ARTNET_IPPROG_REPLY->subnet[3] = glb.conf.uip_subnet[1] >> 8;

    ARTNET_IPPROG_REPLY->port = glb.conf.artnet_node.port;
    ARTNET_IPPROG_REPLY->status = 0;
    memset(ARTNET_IPPROG_REPLY->nu2, 0, 7);
    memset(ARTNET_IPPROG_REPLY->nu1, 0, 4);

    return(sizeof(struct artnet_ip_prog_reply_tp));
}

void artnet_handle_dmx(uint8_t * data)
{
    uint16_t dmx_len = HTONS(ARTNET_DMX->length);

//  DEBUG(("DMX: %d\r\n", dmx_len));
    glb.dmx_mode = 1;
    if ( (dmx_len >= 2) && (dmx_len <=512)) {
        DMX_SetRaw(ARTNET_DMX->data, dmx_len);
    }
}


uint16_t artnet_parser(uint8_t * data, uint16_t datalen) 
{
    uint16_t retlen = 0;
    DEBUG(("opcode: %04X\r\n", ARTNET_HEADER->opCode));

    if (ARTNET_HEADER->opCode == ARTNET_OPCODE_POLL) {
        uip_udp_send(artnet_get_poll_reply(data));
    }
    else if (ARTNET_HEADER->opCode == ARTNET_OPCODE_IPPROG) {
        uip_udp_send(artnet_handle_ipprog_reply(data));
    }
    else if (ARTNET_HEADER->opCode == ARTNET_OPCODE_DMX) {
        artnet_handle_dmx(data);
    }
    else if (ARTNET_HEADER->opCode == ARTNET_OPCODE_ADDRESS) {
        artnet_handle_address(data);
    }
    return (retlen);
}

