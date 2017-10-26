/*
 * gpio.cpp
 *
 * Created: 25/10/2017 2:24:40
 *  Author: jlblanco
 */ 

#include "gpio.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Mapping between pin numbers and actual AVR registers:
// For: 
// * "Atmega{16,32,64,128}4": Ports A, B, C, D. All have full 8 external pins
const uint16_t PROGMEM port_to_mode_PGM[] = {
	0,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	0,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	0,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

void gpio_pin_mode(const uint8_t pin_no, pin_mode_t pin_mode)
{
	const uint8_t port = digitalPinToPort(pin_no);
	if (!port) return;

	const uint8_t mask = digitalPinToBitMask(pin_no);
	volatile uint8_t *ptr_ddr = portModeRegister(port);
	volatile uint8_t *ptr_out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();
	switch (pin_mode)
	{
	case INPUT:
		*ptr_ddr &= ~mask;
		*ptr_out &= ~mask;
		break;
	case INPUT_PULLUP:
		*ptr_ddr &= ~mask;
		*ptr_out |= mask;
		break;
	case OUTPUT:
		*ptr_ddr |= mask;
		break;
	};
	SREG = oldSREG; // restore interrupts, if enabled
}

void gpio_pin_write(const uint8_t pin_no, bool value)
{
	const uint8_t port = digitalPinToPort(pin_no);
	if (!port) return;

	const uint8_t mask = digitalPinToBitMask(pin_no);
	volatile uint8_t *ptr_out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();
	if (value)
	     *ptr_out |= mask;
	else *ptr_out &= ~mask;

	SREG = oldSREG; // restore interrupts, if enabled
}

bool gpio_pin_read(const uint8_t pin_no)
{
	const uint8_t port = digitalPinToPort(pin_no);
	if (!port) return false;

	const uint8_t mask = digitalPinToBitMask(pin_no);
	volatile uint8_t *ptr_in = portInputRegister(port);

	return (*ptr_in & mask)!=0;
}


