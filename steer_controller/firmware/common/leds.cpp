/*
 * leds.cpp
 *
 * Created: 25/10/2017 2:08:50
 *  Author: jlblanco
 */ 

#include "../config.h"
#include "leds.h"
#include "delays.h"
#include <avr/io.h>

void LED_ON()
{
	sbi(LED_DDR,LED_PIN_IDX);
	cbi(LED_PORT,LED_PIN_IDX);
}

void LED_OFF()
{
	sbi(LED_DDR,LED_PIN_IDX);
	sbi(LED_PORT,LED_PIN_IDX);
}


void flash_led(int ntimes, int nms)
{
	for (int i=0;i<ntimes;i++)
	{
		LED_ON();  delay_ms(nms);
		LED_OFF(); delay_ms(nms);
	}
}