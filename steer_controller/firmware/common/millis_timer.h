/*
 * millis_timer.h
 * Defines a 1ms system clock using 8-bit TIMER2
 *
 * Created: 25/10/2017 1:51:17
 *  Author: jlblanco
 */ 

#pragma once

#include <stdint.h>  // uint8_t, etc.

/** Returns the elapsed milliseconds since boot */
uint32_t millis();

/** Must be called at program startup */
void millis_init();

