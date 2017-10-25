/*****************************************************************************************
 FILE: UART.h
 PURPOSE: Provide access to USB FIFO interface chip FT245BM / FT232 / standard UART

Jose Luis Blanco Claraco (C) 2005-2017
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/

#pragma once

#include <avr/io.h>
#include <stdint.h>

/** Access to UART0 */
namespace UART
{
	inline bool ReadByte( uint8_t &data )
	{
		if ( UCSR0A & (1<<RXC0) )
		{
			data = UDR0;
			return true;
		}

		uint8_t c=0xFF;
		while (c--)
			if ( UCSR0A & (1<<RXC0) )
			{
				data = UDR0;
				return true;
			}
		return false; // timeout
	}
	
	/** Reads one byte, blocking until it's received without timeout */
	uint8_t ReadByte_blocking();

	/** Blocks until one line of text is received (ended in '\r' or '\n'). Line length is returned. */
	uint8_t ReadLine_blocking(uint8_t*buf, uint8_t bufSize);

	void WriteByte( uint8_t data);
	void Write( const uint8_t *data, uint16_t nBytes);

	void WriteString( const char *str);

	// Resets the received buffer:
	void ResetReceiver();

	// Configure: sel_parity = 0:N,2:even,3:odd
	void Configure(uint32_t baud_rate, char sel_parity = 0, bool rxen = true,bool txen = true,bool twostopbits=false);
};
