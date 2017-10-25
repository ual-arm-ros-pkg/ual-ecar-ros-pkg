/*
 * millis_timer.cpp
 *
 * Created: 25/10/2017 1:51:29
 *  Author: jlblanco
 */ 

#include "millis_timer.h"

#include <avr/interrupt.h>

volatile uint32_t timer_ms=0;

// Handle the Timer 2 events:
ISR(TIMER2_COMP_vect)
{
	++timer_ms;
}

/** Returns the elapsed milliseconds since boot */
uint32_t millis()
{
	cli();
	const uint32_t ret = timer_ms;
	sei();
	return ret;
}


/** Must be called at program startup */
void millis_init()
{
	timer_ms=0;
	// Timer2: 8bits,
	// prescaler = 8
	// 1 overflow with compare = (1+199): 8* 200 / 16MHz = 100 us
	
	OCR2A = 199;
	// Mode 2: WGM21=1, WGM20=0
	TCCR2A = (1<<WGM21) | (2 /*prescaler=8*/);
	
	// Enable interrupt:
	TIMSK2 |= (1<<OCIE2A);
}