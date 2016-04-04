#ifndef UART_H__
#define UART_H__

#include "common.h"

#define UART_BUF_SIZE_TX    512
#define UART_BUF_SIZE_RX    100

typedef struct {
    uint8_t     txBuffer[UART_BUF_SIZE_TX];     // tx data buffer. The buffer is cyclic 
    uint16_t    tx_index;                       // tx write index, points the next free byte to write the data to be transmitted
    uint16_t    tx_length;
    uint8_t     tx_wait;
    uint8_t     tx_rs485_enable_wait;           // timer for wait 485 enable
    uint8_t     tx_rs485_disable_wait;          // timer for wait 485 disable
    uint8_t     rxBuffer[UART_BUF_SIZE_RX];     // Cyclic buffer space
    uint16_t    rx_index;                       // rx write index, points the next free byte to write the incoming data from UART    
    uint8_t     rx_flag;                        // At leat one character to be read
    uint16_t    rx_timeSinceLastByte;           // How many msec have passed since the last reception. 0: not data, 1: set on every RX byte, >1 ms with no data reception
} tp_uart;

extern tp_uart glb_uart_buffer;

void        UART_Init(uint32_t baudrate);
uint16_t    UART_sendchar(uint8_t ch) ;
uint16_t    UART_SendBufferLen(unsigned char * buffer, uint16_t bufferlen);
uint32_t    UART_getkey(void);

#endif  //UART_H__
