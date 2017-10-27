/*****************************************************************************************
 FILE: CUART.cpp
 PURPOSE: Provide access to USB FIFO interface chip FT245BM / FT232 / standard UART

Jose Luis Blanco Claraco (C) 2005-2014
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/

// Headers
#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <string.h>  // strlen()

#include "uart.h"
#include "../config.h"

#warning Use interrupts

#include <avr/interrupt.h>

#define UART_RX_BUFFER_BITS 5 // 2^5 = 64 bytes
#define UART_RX_BUFFER_LEN  (1<<UART_RX_BUFFER_BITS)
#define UART_RX_BUFFER_MASK ((1<<UART_RX_BUFFER_BITS)-1)

uint8_t uart_rx_buffer[UART_RX_BUFFER_LEN];
uint8_t uart_rx_buffer_write_index = 0;
uint8_t uart_rx_buffer_read_index = 0;

// Handle the UART RX events:
ISR(USART0_RX_vect)
{
	const uint8_t rx_b = UDR0;
	uart_rx_buffer[uart_rx_buffer_write_index++] = rx_b;
	// Circular buffer index:
	uart_rx_buffer_write_index = uart_rx_buffer_write_index & UART_RX_BUFFER_MASK;
}

namespace UART
{
	
unsigned char ReadByte_blocking()
{
	unsigned char status, res;
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
		;
	/* Get status and 9th bit, then data */
	/* from buffer */
	status = UCSR0A;
	res = UDR0;
	/* If error, return -1 */
	if ( status & ((1<<FE0)|(1<<DOR0)|(1<<UPE0)) )
		return -1;

	return res;	
}

/** Blocks until one line of text is received (ended in '\r' or '\n'). Line length is returned. */
uint8_t ReadLine_blocking(char*buf, unsigned char bufSize)
{
	uint8_t len = 0;
	while (len<bufSize)
	{
		unsigned char b = ReadByte_blocking();
		if (b=='\n' || b=='\r')
			break;
		buf[len++]=(char)b;		
	}
	buf[len]='\0';
	return len;		
}



/*-----------------------------------------------------------------
void CUART::Configure( bool RXEN, bool TXEN, char parity, bool stop_2bits, uint16_t baud_divisor )
	Configure the UART. sel_parity = 0:N,2:even,3:odd
  -----------------------------------------------------------------*/
void Configure(uint32_t USART_BAUDRATE, char sel_parity, bool rxen,bool txen,bool twostopbits)
{
	// Define baud rate
	const uint16_t baud_div = (((F_CPU / (USART_BAUDRATE * 8UL))) - 1);
	
	// A
	UCSR0A = (1<<U2X0); // U2X: Prescaler=8
	
	// B
	uint8_t b = 1 << RXCIE0;	// enable received data interrupt
	if (rxen) b|=1 << RXEN0;
	if (txen) b|=1 << TXEN0;
	UCSR0B = b; 

	// C
	b = 0x86;		// 8 bits words
	b |= ((sel_parity & 0x03) << UPM00);
	if (twostopbits) b|= 1 << USBS0;
	UCSR0C = b;
	
	// Baud rate:
	UBRR0 = baud_div;
}



/*-----------------------------------------------------------------
void 	UART::ResetReceiver()
	Empty received queue
  -----------------------------------------------------------------*/
void 	ResetReceiver()
{ 
	register unsigned char b = UDR0;
}


/*-----------------------------------------------------------------
void CUART::WriteByte( unsigned char data)
	Write a byte to UART. 
  -----------------------------------------------------------------*/
void WriteByte( register unsigned char data)
{
	while ( 0 == (UCSR0A & (1 << UDRE0)) ) {};	// wait until free
	UDR0 = data;
}

/*-----------------------------------------------------------------
	Write an array of bytes:
  -----------------------------------------------------------------*/
void Write( const uint8_t *data, uint16_t nBytes)
{
	for (uint16_t i = 0;i<nBytes;i++)
		WriteByte( data[i] );
}

void WriteString( const char *str)
{
	Write((unsigned char*)str, strlen(str));
}



};


