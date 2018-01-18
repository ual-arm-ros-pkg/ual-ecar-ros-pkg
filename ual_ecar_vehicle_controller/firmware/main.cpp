/*****************************************************************************************
 FILE: main.cpp
 PURPOSE: Main entry point of the eCar steering controller firmware

Jose Luis Blanco Claraco (C) 2017
Universidad de Almeria
****************************************************************************************/

#include "libclaraquino/uart.h"
#include "libclaraquino/leds.h"
#include "libclaraquino/delays.h"
#include "libclaraquino/millis_timer.h"
#include "vehicle_controller_declarations.h"

#include <avr/interrupt.h> // sei()

void processCPUStats(const uint32_t dt);

int main(void)
{
	// ================== Setup hardware ==================
	UART::Configure(500000);
	millis_init();

	// Enable interrupts:
	sei();

	flash_led(3,100);

	// Make sure all outputs are in a safe state:
	enableSteerController(false);
	enableThrottleController(false);
	initSensorsForController();

// 	#warning QUITAR! PRUEBA!
// 	enableSteerController(false);
// 	setSteerControllerSetpoint_Steer(100);
// 	setSteerOpenLoopSetpoint_Steer(-100);

	uint32_t t_ini = millis();

	// ============== Infinite loop ====================
	while(1)
	{
		// ---- Run scheduled tasks ----------
		processIncommingPkts();
		processADCs();
		processEncoders();
		processEMS22A();
		processThrottleController();
		processSteerController();
		process_timeouts(); // Handle possible timeouts of previous commands:

		const uint32_t t_end = millis();
		const uint32_t dt = t_end-t_ini; // by computing it here, we also count the cost of processCPUStats()
		t_ini = millis();

		// ---- CPU busy time stats ----------
		processCPUStats(dt);
	}
}

TFrame_CPU_USAGE_STATS  s;
void processCPUStats(const uint32_t dt)
{
	// Accumulate stats:
	s.payload.loop_average_time+=dt;
	if (dt>s.payload.loop_max_time) s.payload.loop_max_time=dt;
	if (dt<s.payload.loop_min_time) s.payload.loop_min_time=dt;

	// Decimate the number of msgs sent to the PC:
	const uint32_t CPU_STATS_DECIMATE = 10000;
	static uint32_t decim0 = 0;
	if (++decim0>CPU_STATS_DECIMATE)
	{
		decim0=0;

		// Send to main PC?
		s.payload.loop_average_time /= CPU_STATS_DECIMATE;
		s.payload.timestamp_ms_tenths = millis();
		s.calc_and_update_checksum();
		s.payload.clear();

		UART::Write((uint8_t*)&s,sizeof(s));
		// reset
		s.payload.clear();
	}
}
