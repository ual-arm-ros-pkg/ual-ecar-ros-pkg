/*
 * config.h
 *
 * Configuration of the PCB board and microcontroller
 *
 * Created: 25/10/2017 1:30:55
 *  Author: jlblanco
 */ 

#pragma once

// XTAL frequency: 20 MHz
#define F_CPU 20000000UL

// Where LEDs are connected (They are supposed to be VCC-tied)
#define LED_PORT    PORTD  // LED1 = PD5
#define LED_DDR     DDRD
#define LED_PIN_IDX 5

#define sbi(port,bit)  (port |=  ( 1 << bit ))
#define cbi(port,bit)  (port &= ~( 1 << bit ))

