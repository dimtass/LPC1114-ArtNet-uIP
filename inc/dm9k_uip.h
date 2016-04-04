/*==============================================================================
|                     dm9k_uip.h: Version 0.04 06/13/2006                      |
|                                                                              |
|          Davicom DM9000A uP NIC fast Ethernet driver for uIP.                |
|                                                                              |
|    This program is free software; you can redistribute it and/or modify it   |
|under the terms of the GNU General Public License as published by the Free    |
|Software Foundation; either version 2 of the License, or (at your option) any |
|later version.                                                                |
|                                                                              |
| This program is distributed in the hope that it will be useful,but WITHOUT   |
|ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS |
|FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.|
|                                                                              |
|    (C)Copyright 1997-1998 DAVICOM Semiconductor,Inc. All Rights Reserved.    |
|============================================================================= |
| China Agent:                        |||¤j³°¥N²z°S?G                          |
|   AXW Technology Limited.           |||  ·R?Y¤e¬µ§?¦³­­¤½¥q                  |
|   TEL : +86 (755) 8368-8556         |||  ?q???G+86 (755) 8368-8556           |
|         +86 (755) 8368-8967         |||        +86 (755) 8368-8967           |
|   FAX : +86 (755) 8377-8765         |||  ¶??u?G+86 (755) 8377-8765           |
|   MAIL: sales@axwdragon.com         |||  ¶l?q?Gsales@axwdragon.com           |
==============================================================================*/

#ifndef _DM9K_H_
#define _DM9K_H_


#include "common.h"

#define UIP_ETHADDR0					0xFF
#define UIP_ETHADDR1					0xFF
#define UIP_ETHADDR2					0xFF
#define UIP_ETHADDR3					0xFF
#define UIP_ETHADDR4					0xFF
#define UIP_ETHADDR5					0xFE

/*************************/ 
#define NCR_EXT_PHY		(1<<7) 
#define NCR_WAKEEN		(1<<6) 
#define NCR_FCOL		(1<<4) 
#define NCR_FDX			(1<<3) 
#define NCR_LBK			(3<<1) 
#define NCR_RST			(1<<0) 
 
#define NSR_SPEED		(1<<7) 
#define NSR_LINKST		(1<<6) 
#define NSR_WAKEST		(1<<5) 
#define NSR_TX2END		(1<<3) 
#define NSR_TX1END		(1<<2) 
#define NSR_RXOV		(1<<1) 
 
#define TCR_TJDIS		(1<<6) 
#define TCR_EXCECM		(1<<5) 
#define TCR_PAD_DIS2	(1<<4) 
#define TCR_CRC_DIS2	(1<<3) 
#define TCR_PAD_DIS1	(1<<2) 
#define TCR_CRC_DIS1	(1<<1) 
#define TCR_TXREQ		(1<<0) 
 
#define TSR_TJTO		(1<<7) 
#define TSR_LC			(1<<6) 
#define TSR_NC			(1<<5) 
#define TSR_LCOL		(1<<4) 
#define TSR_COL			(1<<3) 
#define TSR_EC			(1<<2) 
 
#define RCR_WTDIS		(1<<6) 
#define RCR_DIS_LONG	(1<<5) 
#define RCR_DIS_CRC		(1<<4) 
#define RCR_ALL			(1<<3) 
#define RCR_RUNT		(1<<2) 
#define RCR_PRMSC		(1<<1) 
#define RCR_RXEN		(1<<0) 
 
#define RSR_RF			(1<<7) 
#define RSR_MF			(1<<6) 
#define RSR_LCS			(1<<5) 
#define RSR_RWTO		(1<<4) 
#define RSR_PLE			(1<<3) 
#define RSR_AE			(1<<2) 
#define RSR_CE			(1<<1) 
#define RSR_FOE			(1<<0) 
 
#define FCTR_HWOT(ot)	(( ot & 0xf ) << 4 ) 
#define FCTR_LWOT(ot)	( ot & 0xf ) 
 
#define IMR_PAR			(1<<7) 
#define IMR_ROOM		(1<<3) 
#define IMR_ROM			(1<<2) 
#define IMR_PTM			(1<<1) 
#define IMR_PRM			(1<<0) 
 
 
#define NCR_set         0x00 
#define TCR_set         0x00 
#define TX_REQUEST      0x01  /* TCR REG. 02 TXREQ Bit [0] = 1 polling Transmit Request command */ 
#define TCR_long        0x40  /* packet disable TX Jabber Timer */ 
#define RCR_set         0x30  /* skip CRC_packet and skip LONG_packet */ 
#define RX_ENABLE       0x01  /* RCR REG. 05 RXEN Bit [0] = 1 to enable RX machine */ 
#define RCR_long        0x40  /* packet disable RX Watchdog Timer */ 
#define PASS_MULTICAST  0x08  /* RCR REG. 05 PASS_ALL_MULTICAST Bit [3] = 1: RCR_set value ORed 0x08 */ 
#define BPTR_set        0x3F  /* BPTR REG. 08 RX Back Pressure Threshold: High Water Overflow Threshold setting 3KB and Jam_Pattern_Time = 600 us */ 
#define FCTR_set        0x5A  /* FCTR REG. 09 High/ Low Water Overflow Threshold setting 5KB/ 10KB */ 
#define RTFCR_set       0x29  /* RTFCR REG. 0AH RX/TX Flow Control Register enable TXPEN + BKPM(TX_Half) + FLCE(RX) */ 
#define ETXCSR_set      0x83  /* Early Transmit Bit [7] Enable and Threshold 0~3: 12.5%, 25%, 50%, 75% */ 
#define INTR_set        0x81  /* IMR REG. FFH: PAR +PRM, or 0x83: PAR + PRM + PTM */ 
#define PAR_set         0x80  /* IMR REG. FFH: PAR only, RX/TX FIFO R/W Pointer Auto Return enable */ 
 
#define PHY_reset       0x8000  /* PHY reset: some registers back to default value */ 
#define PHY_txab        0x05e1  /* set PHY TX advertised ability: Full-capability + Flow-control (if necessary) */ 
#define PHY_mode        0x3100  /* set PHY media mode: Auto negotiation (AUTO sense) */ 
 
#define STD_DELAY       20      /* standard delay 20 us */ 
 
#define DMFE_SUCCESS    0 
#define DMFE_FAIL       1 
 
#define TRUE            1 
#define FALSE           0 
 
/*************************/ 

/* DM9000 REGISTER LIST */
#define DM9000_REG_NCR        0x00
#define DM9000_REG_NSR        0x01
#define DM9000_REG_TCR        0x02
#define DM9000_REG_TSR1       0x03
#define DM9000_REG_TSR2       0x04
#define DM9000_REG_RCR        0x05
#define DM9000_REG_RSR        0x06
#define DM9000_REG_ROCR       0x07
#define DM9000_REG_BPTR       0x08
#define DM9000_REG_FCTR       0x09
#define DM9000_REG_FCR        0x0A
#define DM9000_REG_EPCR       0x0B
#define DM9000_REG_EPAR       0x0C
#define DM9000_REG_EPDRL      0x0D
#define DM9000_REG_EPDRH      0x0E
#define DM9000_REG_WAR        0x0F
#define DM9000_REG_PAR        0x10
#define DM9000_REG_MAR        0x16
#define DM9000_REG_MAR7       0x1D
#define DM9000_REG_GPCR       0x1E
#define DM9000_REG_GPR        0x1F
#define DM9000_REG_VID_L      0x28
#define DM9000_REG_VID_H      0x29
#define DM9000_REG_PID_L      0x2A
#define DM9000_REG_PID_H      0x2B
#define DM9000_REG_CHIPR      0x2C
#define DM9000_REG_TCR2       0x2D
#define DM9000_REG_OTCR       0x2E
#define DM9000_REG_SMCR       0x2F
#define DM9000_REG_ETXCSR     0x30
#define DM9000_REG_TCSCR      0x31
#define DM9000_REG_RCSCSR     0x32
#define DM9000_REG_MRCMDX     0xF0
#define DM9000_REG_MRCMD      0xF2
#define DM9000_REG_MRRL       0xF4
#define DM9000_REG_MRRH       0xF5
#define DM9000_REG_MWCMDX     0xF6
#define DM9000_REG_MWCMD      0xF8
#define DM9000_REG_MWRL       0xFA
#define DM9000_REG_MWRH       0xFB
#define DM9000_REG_TXPLL      0xFC
#define DM9000_REG_TXPLH      0xFD
#define DM9000_REG_ISR        0xFE
#define DM9000_REG_IMR        0xFF


/* ¬?Gf§»³]?m */
#define DM9000_ID             0x90000A46
#define DM9000_BYTE_MODE      0x01
#define DM9000_WORD_MODE      0x00
#define DM9000_PHY            0x40
#define DM9000_PKT_RDY        0x01
#define DM9000_PKT_NORDY      0x00                

#define DM9000_RX_INTR        0x01                /* ±µ¦¬¤¤?_§P?_ bit */
#define DM9000_TX_INTR        0x02                /* ¶?°e¤¤?_§P?_ bit */
#define DM9000_OVERFLOW_INTR  0x04                /* ¤?¦s·?¥X¤¤?_§P?_ bit */
#define DM9000_LINK_CHANG     0x20                /* ³s±µ??°?¤¤?_§P?_ bit */


#define DM9000_REG_BMCR       0x00
#define DM9000_REG_RESET      0x03                /* T???F?FG????????FG */ 
#define DM9000_PHY_ON         0x00                /* ??¶¨ PHY ???t */ 
#define DM9000_PHY_SET        0x00                /* ³]©w PHY ¶}±? */
#define DM9000_PHY_OFF        0x01                /* ³]©w PHY Gf³¬ */
#define DM9000_RCR_SET        0x31                /* ³]©w ±µ¦¬¥\?? (¤£¦¬ CRC ¤? ¶W??¥]) */
#define DM9000_TCR_SET        0x01                /* ³]©w ¶?°e¥\?? */
#define DM9000_RCR_OFF        0x00                /* ³]©w ±µ¦¬¥\??GfGf³¬³]?m */
#define DM9000_BPTR_SET       0x37                /* ³]©w Back Pressure ±?¥s³]?m */
#define DM9000_FCTR_SET       0x38                /* ³]©w Flow Control ±?¥s³]?m */
#define DM9000_TCR2_SET       0x80                /* ³]?m LED ??¥???¦? */
#define DM9000_OTCR_SET       0x80                /* ³]?m DM9000 ¤u§@?W²v 0x80 = 100Mhz */
#define DM9000_ETXCSR_SET     0x83                /* ³]?m Early Tramsmit ±?¥s³]?m */
#define DM9000_FCR_SET        0x28                /* ¶}±? ?tµ?¬y±±¥\??³]?m */
#define DM9000_TCSCR_SET      0x07                /* ³]©w CHECKSUM ¶?°e?B?ß ³]?m */
#define DM9000_RCSCSR_SET     0x03                /* ³]©w CHECKSUM ±µ¦¬??¬d ³]?m */
#define DM9000_IMR_SET        0x81                /* ³]©w ±?¥?¤¤?_¨??? ±?¥s³]?m */
#define DM9000_IMR_OFF        0x80                /* ³]©w Gf³¬¤¤?_¨??? ±?¥s³]?m */


#define PHY_REG_CTRL        0x00
#define PHY_REG_STATUS      0x01
#define PHY_REG_PHYID1      0x02
#define PHY_REG_PHYID2      0x03

/* EXPORTED SUBPROGRAM SPECIFICATIONS */
void etherdev_init(void);
void etherdev_send(void);
unsigned short etherdev_poll(void);
void etherdev_chkmedia(void);
void etherdev_set_mac_address(uint8_t * MAC);
void etherdev_get_mac_address(uint8_t ** MAC);
void dm9k_udelay(uint16_t time);
void dm9k_reset(void);
#endif /* _DM9K_H_ */
