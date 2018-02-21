/*
 * firmware.cpp
 *
 * Created: 04/10/2017 10:46:02
 * Author : Francisco Jose Manas
 */ 

#include "libclaraquino/uart.h"
#include "libclaraquino/leds.h"
#include "libclaraquino/delays.h"
#include "libclaraquino/millis_timer.h"
#include "batterycharge_declarations.h"

#include <avr/interrupt.h> // sei()

int main(void)
{
	// ================== Setup hardware ==================
	UART::Configure(500000);
	millis_init();

	// Enable interrupts:
	sei();

/*	flash_led(3,100);*/

	process_batery_init();
	
	
	
	

	// ============== Infinite loop ====================
	while(1)
	{
		// ---- Run scheduled tasks ----------
		processIncommingPkts();
		processBattery();
	}
}
