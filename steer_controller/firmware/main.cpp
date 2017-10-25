/*****************************************************************************************
 FILE: main.cpp
 PURPOSE: Main entry point of the eCar steering controller firmware

Jose Luis Blanco Claraco (C) 2017
Universidad de Almeria
****************************************************************************************/

#include "common/uart.h"
#include "common/leds.h"
#include "common/millis_timer.h"
#include "steer_controller_declarations.h"

#include <avr/interrupt.h> // sei()


int main(void)
{
	// ================== Setup hardware ==================
	UART::Configure(115200);
	millis_init();
	
	// Enable interrupts:
	sei();

	// ============== Infinite loop ====================
	while(1)
	{
		processIncommingPkts();
		processADCs();
		processEncoders();
		processEMS22A();

		// Handle possible timeouts of previous commands:
		process_timeouts();
	}
}
