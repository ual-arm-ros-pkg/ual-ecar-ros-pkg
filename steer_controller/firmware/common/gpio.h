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
	OUTPUT
};

void gpio_pin_mode(const uint8_t pin_no, pin_mode_t pin_mode);
void gpio_pin_write(const uint8_t pin_no, bool value);
bool gpio_pin_read(const uint8_t pin_no);


