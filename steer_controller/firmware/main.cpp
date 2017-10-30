/*****************************************************************************************
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
#include "common/pwm.h"

#include <avr/interrupt.h> // sei()

int main(void)
{
	// ================== Setup hardware ==================
	UART::Configure(500000);
	millis_init();

	// Enable interrupts:
	sei();

	flash_led(3,100);

	pwm_init(PWM_TIMER0, PWM_PRESCALER_1 );
	pwm_set_duty_cycle(PWM_TIMER0, PWM_PIN_OCnA , 0x80);

	// ============== Infinite loop ====================
	while(1)
	{
		processIncommingPkts();
		processADCs();
		processEncoders();
		processEMS22A();

		//processSteerControl();

		// Handle possible timeouts of previous commands:
		process_timeouts();
	}
}
