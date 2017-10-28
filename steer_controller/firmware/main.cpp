﻿/*****************************************************************************************
 FILE: main.cpp
 PURPOSE: Main entry point of the eCar steering controller firmware

Jose Luis Blanco Claraco (C) 2017
Universidad de Almeria
****************************************************************************************/

#include "common/uart.h"
#include "common/leds.h"
#include "common/delays.h"
#include "common/millis_timer.h"
#include "steer_controller_declarations.h"

#include <avr/interrupt.h> // sei()
#include <stdio.h>


int main(void)
{
	// ================== Setup hardware ==================
	UART::Configure(500000);
	millis_init();

	// Enable interrupts:
	sei();

	flash_led(3,100);

	// ============== Infinite loop ====================
	while(1)
	{
#if 0
		processIncommingPkts();
		processADCs();
		processEncoders();
		processEMS22A();

		// Handle possible timeouts of previous commands:
		process_timeouts();
#endif

		delay_ms(1000);
		char str[100];
		sprintf(str,"Hello world! tim=%lu for_read=%u\n", millis(), UART::AvailableForRead() );
		UART::WriteString(str);
	}
}
