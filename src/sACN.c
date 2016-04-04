
#include "common.h"
#include "sACN.h"

#define DEBUG(X)   TRACEL(TRACE_LEVEL_ACN, X)


const uint8_t   acn_packet_id[12] = {0x41, 0x53, 0x43, 0x2D, 0x45, 0x31, 0x2E, 0x31, 0x37, 0x00, 0x00, 0x00};


void acn_init(void)
{
    glb.conf.acn_port = ACN_DEF_UDP_PORT;
}


uint16_t acn_parser(uint8_t * data, uint16_t datalen) 
{
    uint16_t retlen = 0;
    DEBUG(("ACN packet!\r\n"));
//  DEBUG(("acn opcode: %04X\r\n", ARTNET_HEADER->opCode));
//
//  if (ARTNET_HEADER->opCode == ARTNET_POLL) {
//      uip_udp_send(artnet_get_poll_reply(data));
//  }

    return (retlen);
}
