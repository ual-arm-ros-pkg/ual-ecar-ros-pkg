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
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/signal.h>
#include <string.h>

#include "CUART.h"
#include "LEDs.h"

#include "public_include/steering-control-firmware2pc-structs.h"


uint8_t UART_LAST_RX_CMD_TEMP[UART_LAST_RX_CMD_SIZE];  // Temporary rx buffer (in progress)
uint8_t UART_LAST_RX_CMD[UART_LAST_RX_CMD_SIZE];  // Temporary rx buffer (in progress)
unsigned int UART_LAST_RX_CMD_INDEX = 0;
uint16_t UART_LAST_RX_CMD_LEN=0;
volatile bool UART_LAST_RX_CMD_NEW = false;


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
void Configure(bool rxen,bool txen,bool twostopbits,char sel_parity,uint16_t baud_div)
{
	uint8_t	b;
	
	// A
	UCSR0A = (1<<U2X0); // U2X: Prescaler=8
	
	// B
	b = 0x00;	// Dont enable received data interrupt
	if (rxen) b|=1 << RXEN0;
	if (txen) b|=1 << TXEN0;	
	UCSR0B = b; 

	// C
	b = 0x86;		// 8 bits words
	b |= ((sel_parity & 0x03) << UPM00);
	if (twostopbits) b|= 1 << USBS0;
	UCSR0C = b;
	
	// Baud rate:
	//UBRR0H = (baud_div >> 8) & 0x0F;	
	//UBRR0L = baud_div & 0xFF;	
	UBRR0 = baud_div;
}



/*-----------------------------------------------------------------
void 	UART::ResetReceiver()
	Empty received queue
  -----------------------------------------------------------------*/
void 	ResetReceiver()
{ 
	UDR0;
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
void Write( register unsigned char *data, register uint16_t nBytes)
{
	for (register uint16_t i = 0;i<nBytes;i++)
		WriteByte( data[i] );
}

void WriteString( const char *str)
{
	Write((unsigned char*)str, strlen(str));
}


/** Writes a string with a prefix that informs the receiver of the expected length of the data block. */
void WriteStringFramed( const char *str)
{
	uint16_t len = strlen(str);
	//char buf[8];
	//sprintf(buf,"%04X ",len);  // 72us
	//Write((unsigned char*)buf, 5);

	WriteByte(STEERCONTROL_COMMS_FRAME_START_FLAG);
	Write((unsigned char*)&len, 2);
	Write((unsigned char*)str, len);	
	WriteByte(STEERCONTROL_COMMS_FRAME_END_FLAG);
}

/** Writes a binary string with a prefix that informs the receiver of the expected length of the data block. */
void WriteBinaryFrame( const uint8_t *data, uint16_t len)
{	
	WriteByte(STEERCONTROL_COMMS_FRAME_START_FLAG);
	Write((unsigned char*)&len, 2);
	Write((unsigned char*)data, len);
	WriteByte(STEERCONTROL_COMMS_FRAME_END_FLAG);
}

// Inits the interrupt, event-based system:
void InitInterruptBasedCommandReceiver()
{
	UART_LAST_RX_CMD_NEW=false;
	UART_LAST_RX_CMD_LEN=0;
	UART_LAST_RX_CMD_INDEX=0;
	
	sbi(UCSR0B, RXCIE0);	
}

#if 0
//===========================  TEXT BASED UART RX  ===========================

// Handle the Timer 2 events:
SIGNAL(USART0__RX_vect)
{
	// Get byte from UART:
	const uint8_t rx = UDR0;
	
	const bool is_newline_char = (rx=='\n' || rx=='\r');	
	// If the buffer is still empty, ignore whitespaces:
	if (UART_LAST_RX_CMD_INDEX==0 && is_newline_char) 
		return;

	// End condition?
	if (UART_LAST_RX_CMD_INDEX>=UART_LAST_RX_CMD_SIZE || is_newline_char)
	{
		// Copy to definitive buffer and reset temp buffer:
		if (!UART_LAST_RX_CMD_NEW)
		{
			memcpy(UART_LAST_RX_CMD,UART_LAST_RX_CMD_TEMP,UART_LAST_RX_CMD_INDEX);
			UART_LAST_RX_CMD[UART_LAST_RX_CMD_INDEX]='\0';
			UART_LAST_RX_CMD_NEW=true;
			UART_LAST_RX_CMD_INDEX=0;		
		}		
	}
	else
	{
		// Append to buffer:
		UART_LAST_RX_CMD_TEMP[UART_LAST_RX_CMD_INDEX++] = rx;		
	}
}
#else
//===========================  BINARY FRAMED-BASED UART RX  ===========================

void QUEUE_RX_BYTE(const uint8_t rx)
{
	// Append to buffer:
	UART_LAST_RX_CMD_TEMP[UART_LAST_RX_CMD_INDEX++] = rx;
	
	// Checks:
	if (UART_LAST_RX_CMD_TEMP[0]!=STEERCONTROL_COMMS_FRAME_START_FLAG)
	{
		UART_LAST_RX_CMD_INDEX = 0;
	}
	else
	{
		if (UART_LAST_RX_CMD_INDEX>=4)
		{
			// Is frame complete?
			//                                   START_FLAG    LENGTH_FIELD       DATA                     END_FLAG
			//while ( nFrameBytes < (lengthField=(    1        +     2       +   (buf[1] | (buf[2] << 8))) +     1      )  )
			uint16_t LEN = 1 + 2 +1 + ( uint16_t(UART_LAST_RX_CMD_TEMP[1])|( uint16_t(UART_LAST_RX_CMD_TEMP[2])<<8 ));
			if (LEN==0)
			{
				// Reset bad frame:
				UART_LAST_RX_CMD_INDEX=0;
			}
			else
			{
				if (LEN==UART_LAST_RX_CMD_INDEX)
				{
					if (UART_LAST_RX_CMD_TEMP[UART_LAST_RX_CMD_INDEX-1]== STEERCONTROL_COMMS_FRAME_END_FLAG )
					{
						// All OK:
						UART_LAST_RX_CMD_LEN = LEN;
						memcpy(UART_LAST_RX_CMD,UART_LAST_RX_CMD_TEMP+3,LEN);
						UART_LAST_RX_CMD_NEW=true;
						UART_LAST_RX_CMD_INDEX=0;
					}
					else
					{
						// Bad frame:
						UART_LAST_RX_CMD_INDEX=0;
					}
				}
			}
		}
	}		
}
#if!defined(USART0_RX_vect)&&defined(USART0_RX_vect)
#define USART0_RX_vect USART0_RX_vect
#endif 
// Handle the Timer 2 events:
SIGNAL(USART0_RX_vect)
{
	// Get byte from UART:
	const uint8_t rx = UDR0;
	UART::QUEUE_RX_BYTE(rx);
}
#endif




};



