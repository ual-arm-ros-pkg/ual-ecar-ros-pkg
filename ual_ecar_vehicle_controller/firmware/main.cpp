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

#include <avr/interrupt.h> // sei()

void processCPUStats(const uint32_t dt, TFrame_CPU_USAGE_STATS &s);

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
// 	enableSteerController(true);
// 	setSteerControllerSetpoint_Steer(100);

	TFrame_CPU_USAGE_STATS  cpu_busy_stats;
	cpu_busy_stats.payload.clear();
	uint32_t t_ini = millis();

	// ============== Infinite loop ====================
	while(1)
	{
		// ---- Run scheduled tasks ----------
		processIncommingPkts();
		processADCs();
		processEncoders();
		processEMS22A();
		processSteerController();
		process_timeouts(); // Handle possible timeouts of previous commands:

		const uint32_t t_end = millis();
		const uint32_t dt = t_end-t_ini; // by computing it here, we also count the cost of processCPUStats()
		t_ini = millis();

		// ---- CPU busy time stats ----------
		processCPUStats(dt, cpu_busy_stats);
	}
}


void processCPUStats(const uint32_t dt, TFrame_CPU_USAGE_STATS &frame)
{
	auto &s = frame.payload;

	// Accumulate stats:
	s.loop_average_time+=dt;
	if (dt>s.loop_max_time) s.loop_max_time=dt;
	if (dt<s.loop_min_time) s.loop_min_time=dt;

	// Send to main PC?
	if (++s.loop_iterations>=0x100)
	{
		s.loop_average_time /= 0x100;
		s.timestamp_ms_tenths = millis();
		
		frame.calc_and_update_checksum();
		UART::Write((uint8_t*)&frame,sizeof(frame));
		s.clear();
	}
}
