/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low level serial routines
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "LPC11xx.h"                            /* LPC11xx definitions        */
#include "uart.h"

#define AUTOBAUD_ENABLE 0
#define FDR_CALIBRATION 0
#define RS485_ENABLED   0
#define TX_INTERRUPT    0		/* 0 if TX uses polling, 1 interrupt driven. */
#define MODEM_TEST      0

#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)
#define IER_ABEO        (0x01<<8)
#define IER_ABTO        (0x01<<9)


#define IIR_PEND        0x01
#define IIR_RLS         0x03
#define IIR_RDA         0x02
#define IIR_CTI         0x06
#define IIR_THRE        0x01
#define IIR_ABEO        (0x01<<8)
#define IIR_ABTO        (0x01<<9)

#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

/* RS485 mode definition. */
#define RS485_NMMEN		(0x1<<0)
#define RS485_RXDIS		(0x1<<1)
#define RS485_AADEN		(0x1<<2)
#define RS485_SEL             (0x1<<3)
#define RS485_DCTRL		(0x1<<4)
#define RS485_OINV		(0x1<<5)


tp_uart glb_uart_buffer;


volatile uint32_t UARTStatus;
volatile uint8_t  UARTTxEmpty = 1;
volatile uint32_t UARTCount = 0;


uint32_t uart_set_divisors(uint32_t UARTClk, uint32_t baudrate);

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void UART_Init(uint32_t baudrate) {

    volatile uint32_t regVal;
    uint32_t Fdiv;

    UARTTxEmpty = 1;
    UARTCount = 0;

    NVIC_DisableIRQ(UART_IRQn);

    /* configure PINs GPIO1.6, GPIO1.7 for UART */
    LPC_SYSCON->SYSAHBCLKCTRL |= ((1UL <<  6) |   /* enable clock for GPIO      */
                                      (1UL << 16));  /* enable clock for IOCON     */

    LPC_IOCON->PIO1_6  =  (1UL <<  0);            /* P1.6 is RxD                */
    LPC_IOCON->PIO1_7  =  (1UL <<  0);            /* P1.7 is TxD                */

    /* configure UART0 */
    LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 12);    /* Enable clock to UART       */
    LPC_SYSCON->UARTCLKDIV     =  1; // (4UL <<  0);    /* UART clock =  CCLK / 4     */

    //LPC_UART->LCR = 0x83;       // 8 bits, no Parity, 1 Stop bit 
    LPC_UART->LCR = 0x87;       // 8 bits, no Parity, 2 Stop bit   

    if ( uart_set_divisors(SystemCoreClock/LPC_SYSCON->UARTCLKDIV, baudrate) != 1 )
	{
      Fdiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate ;	/*baud rate */
      LPC_UART->DLM = Fdiv / 256;							
      LPC_UART->DLL = Fdiv % 256;
	  LPC_UART->FDR = 0x10;		/* Default */
	}
    else {
        Fdiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate ;	/*baud rate */
        LPC_UART->DLM = Fdiv / 256;							
        LPC_UART->DLL = Fdiv % 256;
    	LPC_UART->FDR = 0x10;		/* Default */
    }
                                
    LPC_UART->LCR = 0x03;       // DLAB = 0                        
    LPC_UART->FCR = 0x07;		// Enable and reset TX and RX FIFO.

    /* Read to clear the line status. */
    regVal = LPC_UART->LSR;

    /* Ensure a clean start, no data in either TX or RX FIFO. */
    while (( LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
    while ( LPC_UART->LSR & LSR_RDR )
    {
        regVal = LPC_UART->RBR;	/* Dump data from RX FIFO */
    }

    
    NVIC_SetPriority(UART_IRQn, 2); // Set interrupt Priority 
    NVIC_EnableIRQ(UART_IRQn);      //Enable interrupt

    //reset buffers
    glb_uart_buffer.rx_index = 0;
    glb_uart_buffer.tx_index = 0;
    glb_uart_buffer.tx_length = 0;

    
    LPC_UART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
}
 



/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
uint16_t UART_sendchar(uint8_t ch) 
{

    if (glb_uart_buffer.tx_length) {
        glb_uart_buffer.txBuffer[glb_uart_buffer.tx_length] = ch;
        glb_uart_buffer.tx_length++;
        if (glb_uart_buffer.tx_length >= UART_BUF_SIZE_TX) {
            glb_uart_buffer.tx_length--;
            glb_uart_buffer.txBuffer[glb_uart_buffer.tx_length - 3] = 'O';
            glb_uart_buffer.txBuffer[glb_uart_buffer.tx_length - 2] = 'V';
            glb_uart_buffer.txBuffer[glb_uart_buffer.tx_length - 1] = 'F';

            // Empty buffer on overflow //DIM_20131024
            glb_uart_buffer.tx_length = 0;
        }
    } else {
        glb_uart_buffer.tx_index = 1;
        glb_uart_buffer.tx_length = 1;
        while (!(LPC_UART->LSR & 0x20));
        LPC_UART->THR = ch;
    }
    return (ch);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
uint32_t UART_getkey(void) 
{

    while (!(LPC_UART->LSR & 0x01));
    return (LPC_UART->RBR);
}


uint16_t UART_SendBufferLen(unsigned char * buffer, uint16_t bufferlen)
{
    uint16_t i;
    glb_uart_buffer.tx_length = 0;
    for (i=0; i<bufferlen; i++) {
        glb_uart_buffer.txBuffer[i] = buffer[i];
    }
    glb_uart_buffer.tx_length = bufferlen;

    glb_uart_buffer.tx_index = 0;
    while (!(LPC_UART->LSR & 0x20));
    LPC_UART->THR = glb_uart_buffer.txBuffer[glb_uart_buffer.tx_index++];
    return(i);
}


__irq void UART_IRQHandler(void) {
    uint8_t IIRValue, LSRValue;
    uint8_t Dummy = Dummy;

    IIRValue = LPC_UART->IIR;

    IIRValue >>= 1;                 // skip pending bit in IIR
    IIRValue &= 0x07;               // check bit 1~3, interrupt identification
    if (IIRValue == IIR_RLS) {      // Receive Line Status
        LSRValue = LPC_UART->LSR;
        // Receive Line Status
        if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
            // There are errors or break interrupt
            // Read LSR will clear the interrupt
            UARTStatus = LSRValue;
            Dummy = LPC_UART->RBR;  // Dummy read on RX to clear interrupt, then bail out */
            return;
        }
    } else if (IIRValue == IIR_RDA) { // Receive Data Available
        // Receive Data Available
        if (glb_uart_buffer.rx_index >= UART_BUF_SIZE_RX ) glb_uart_buffer.rx_index = 0;
        glb_uart_buffer.rxBuffer[glb_uart_buffer.rx_index++] = LPC_UART->RBR;
        glb_uart_buffer.rx_flag = 1;
        glb_uart_buffer.rx_timeSinceLastByte = 1;

    } else if (IIRValue == IIR_CTI) { // Character timeout indicator
        /* Character Time-out indicator */
        UARTStatus |= 0x100;        // Bit 9 as the CTI error

    } else if (IIRValue == IIR_THRE) {    // THRE, transmit holding register empty
        // THRE interrupt
        LSRValue = LPC_UART->LSR;       // Check status in the LSR to see if valid data in U0THR or not
        if (LSRValue & LSR_THRE) {
            UARTTxEmpty = 1;
            if (glb_uart_buffer.tx_index < glb_uart_buffer.tx_length) {
                LPC_UART->THR = glb_uart_buffer.txBuffer[glb_uart_buffer.tx_index++];
                //glb_uart_buffer.tx_index++;
            }
            else {
                glb_uart_buffer.tx_index = glb_uart_buffer.tx_length = 0;
                glb_uart_buffer.tx_wait = 2;
                glb_uart_buffer.tx_rs485_disable_wait = 2;
            }
        } else {
            UARTTxEmpty = 0;
        }
    }
    return;
}

uint32_t uart_set_divisors(uint32_t UARTClk, uint32_t baudrate)
{
  uint32_t uClk;
  uint32_t calcBaudrate = 0;
  uint32_t temp = 0;

  uint32_t mulFracDiv, dividerAddFracDiv;
  uint32_t diviser = 0 ;
  uint32_t mulFracDivOptimal = 1;
  uint32_t dividerAddOptimal = 0;
  uint32_t diviserOptimal = 0;

  uint32_t relativeError = 0;
  uint32_t relativeOptimalError = 100000;

  /* get UART block clock */
  uClk = UARTClk >> 4; /* div by 16 */
  /* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
   * The formula is :
   * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
   * It involves floating point calculations. That's the reason the formulae are adjusted with
   * Multiply and divide method.*/
  /* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
   * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
  for (mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++)
  {
    for (dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++)
    {
      temp = (mulFracDiv * uClk) / ((mulFracDiv + dividerAddFracDiv));
      diviser = temp / baudrate;
      if ((temp % baudrate) > (baudrate / 2))
        diviser++;

      if (diviser > 2 && diviser < 65536)
      {
        calcBaudrate = temp / diviser;

        if (calcBaudrate <= baudrate)
          relativeError = baudrate - calcBaudrate;
        else
          relativeError = calcBaudrate - baudrate;

        if ((relativeError < relativeOptimalError))
        {
          mulFracDivOptimal = mulFracDiv ;
          dividerAddOptimal = dividerAddFracDiv;
          diviserOptimal = diviser;
          relativeOptimalError = relativeError;
          if (relativeError == 0)
            break;
        }
      } /* End of if */
    } /* end of inner for loop */
    if (relativeError == 0)
      break;
  } /* end of outer for loop  */

  if (relativeOptimalError < (baudrate / 30))
  {
    /* Set the `Divisor Latch Access Bit` and enable so the DLL/DLM access*/
    /* Initialise the `Divisor latch LSB` and `Divisor latch MSB` registers */
    LPC_UART->DLM = (diviserOptimal >> 8) & 0xFF;
    LPC_UART->DLL = diviserOptimal & 0xFF;

    /* Initialise the Fractional Divider Register */
    LPC_UART->FDR = ((mulFracDivOptimal & 0xF) << 4) | (dividerAddOptimal & 0xF);
    return( 1 );
  }
  return ( 0 );
}

