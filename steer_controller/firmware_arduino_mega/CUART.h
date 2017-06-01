/*****************************************************************************************
 FILE: UART.h
 Jose Luis Blanco Claraco (C) 2005-2014  
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/
#ifndef UART_H			// For avoiding duplicates definitions
#define UART_H

namespace UART {
	inline bool ReadByte( register unsigned char &data ) {
		if ( UCSR0A & (1<<RXC0) ) {
			data = UDR0;			
			return true;
		}
		register uint8_t c=0xFF;
		while (c--){
			if ( UCSR0A & (1<<RXC0) ){
				data = UDR0;			
				return true;
			}
		}
		return false;	// timeout
	}
	
	/** Reads one byte, blocking until it's received without timeout */
	unsigned char ReadByte_blocking();

	/** Blocks until one line of text is received (ended in '\r' or '\n'). Line length is returned. */
	uint8_t ReadLine_blocking(char*buf, unsigned char bufSize);

	void WriteByte( register unsigned char data);
	void Write( register unsigned char *data, register uint16_t nBytes);

	void WriteString( const char *str);

	/** Writes a string with a prefix that informs the receiver of the expected length of the data block. */
	void WriteStringFramed( const char *str);

	/** Writes a binary string with a prefix that informs the receiver of the expected length of the data block. */
	void WriteBinaryFrame( const uint8_t *data, uint16_t len);

	// Resets the received buffer:
	void 	ResetReceiver();

	// Configure: sel_parity = 0:N,2:even,3:odd
	void Configure(bool rxen,bool txen,bool twostopbits,char sel_parity,uint16_t baud_div);

	// Inits the interrupt, event-based system. Configure() must be also called to enable and config the UART.
	void InitInterruptBasedCommandReceiver();
	
};

#define UART_LAST_RX_CMD_SIZE  100
extern uint8_t  UART_LAST_RX_CMD[UART_LAST_RX_CMD_SIZE];  // rx buffer with the last rx cmd (when UART_LAST_RX_CMD_NEW=true)
extern uint16_t UART_LAST_RX_CMD_LEN; 
extern volatile bool UART_LAST_RX_CMD_NEW;

#endif
