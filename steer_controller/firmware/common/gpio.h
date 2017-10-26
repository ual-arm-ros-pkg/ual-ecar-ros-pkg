/*
 * gpio.h
 *
 * Created: 25/10/2017 2:24:23
 *  Author: jlblanco
 */ 

#pragma once

#include <stdint.h>

enum pin_mode_t
{
	INPUT = 0,
	OUTPUT,
	INPUT_PULLUP
};

void gpio_pin_mode(const uint8_t pin_no, pin_mode_t pin_mode);
void gpio_pin_write(const uint8_t pin_no, bool value);
bool gpio_pin_read(const uint8_t pin_no);


#include <avr/pgmspace.h> // pgm_read_*()

extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

//extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
//extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
//extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Pin numbers: PORTA: 0x10-0x17; PORTB: 0x20-0x27, etc.
inline uint8_t digitalPinToPort(const uint8_t pin_no)
{
	if (!pin_no)
	     return 0;
	else return (0xf0 & pin_no) >> 4;
}

inline uint8_t digitalPinToBitMask(const uint8_t pin_no)
{
	if (!pin_no)
	     return 0;
	else return 1<< (0x0f & pin_no);
}

#define portOutputRegister(port_no) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (port_no))) )
#define portInputRegister(port_no) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (port_no))) )
#define portModeRegister(port_no) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (port_no))) )

#if 0
#define (P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#endif


