/*****************************************************************************************
 FILE: LEDs.h
 PURPOSE: Simple macros for lighting on-off the leds

Jose Luis Blanco Claraco (C) 2005-2014
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/

// Where leds are connected (They are supposed to be VCC-tied) Placa ArduinoMega
#define	LED_PORT		PORTD
#define	LED_DDR			DDRD

// Macros:
#define LED_ON(i)		{ sbi(LED_DDR,i);cbi(LED_PORT,i); }
#define LED_OFF(i)		{ sbi(LED_DDR,i);sbi(LED_PORT,i); }

#define sbi(port,bit)  (port |=  ( 1 << bit ))
#define cbi(port,bit)  (port &= ~( 1 << bit ))

/* macros to set and clear bits in registers */
#define SetBit(sfr, bit) ((sfr) |= (1 << (bit)))
#define ClearBit(sfr, bit) ((sfr) &= ~(1 << (bit)))

