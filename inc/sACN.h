#ifndef SACN_H__
#define SACN_H__
#pragma once

#include "common.h"
#include "uip.h"

#define ACN_PREAMBLE_SIZE   0x0010
#define ACN_POST_AMBLE_SIZE 0x0000
#define ACN_FLAGS           0x07
#define ACN_VECTOR_ROOT     0x00000004
#define ACN_VECTOR_FRAME    0x00000002
#define ACN_VECTOR_DMP      0x02
#define ACN_DEF_PRIORITY    100
#define ACN_ADDR_DATA_TYPE  0xA1
#define ACN_FIRST_PROP_ADDR 0x0000
#define ACN_ADDR_INCR       0x0001
#define ACN_DEF_UDP_PORT    5568



__packed
typedef struct {
    uint16_t    preamble;       // Define RLP Preamble Size. 0x0010
    uint16_t    post_preamble;  // RLP Post-amble Size. 0x0000
    uint8_t     acn_identifier[12]; // Identifies this packet as E1.17
    uint16_t    length;         // b0-b11: PDU length, b12-b15: 0x07
    uint32_t    vector;         // Identifies RLP Data as 1.31 Protocol PDU. Always=0x00000004
    uint8_t     cid[16];        // Sender's CID. Sender's unique ID
} tp_acn_root_layer;

__packed
typedef struct {
    uint16_t    length;         // Low 12 bits = PDU length High 4 bits = 0x7
    uint32_t    vector;         // Identifies 1.31 data as DMP Protocol PDU. Always=0x00000002
    uint8_t     src_name[64];   // User Assigned Name of Source. UTF-8 [UTF-8] encoded string, null-terminated
    uint8_t     priority;       // Data priority if multiple sources. 0-200, default of 100 
    uint16_t    reserved;
    uint8_t     seq_num;        // Sequence Number. To detect duplicate or out of order packets.
    uint8_t     options;        // Options Flags. Bit 7 = Preview_Data, Bit 6 = Stream_Terminated
    uint16_t    universe;       // Universe Number. Identifier for a distinct stream of DMX Data
} tp_acn_framing_layer;

__packed
typedef struct {
    uint16_t    length;         // Low 12 bits = PDU length High 4 bits = 0x7
    uint8_t     vector;         // Identifies DMP Set Property Message PDU. Always=0x02
    uint8_t     addr_data_type; // Identifies format of address and data. Always=0xa1
    uint16_t    first_prop_addr;// First Property Address. Indicates DMX START Code is at DMP address 0. Always=0x0000
    uint16_t    addr_incr;      // Address Increment. Indicates each property is 1 octet. Allways=0x0001
    uint16_t    value_count;    // Property value count. Indicates 1+ the number of slots in packet. Values=0x0001 -- 0x0201
    uint8_t     *values;        // Property values DMX512-A START Code + data. START Code + Data
} tp_acn_dmp_layer;

__packed
typedef struct { 
    tp_acn_root_layer       root;
    tp_acn_framing_layer    frame;
    tp_acn_dmp_layer        dmp;
} tp_sacn_packet;

void acn_init(void);
uint16_t acn_parser(uint8_t * data, uint16_t datalen);


#endif

