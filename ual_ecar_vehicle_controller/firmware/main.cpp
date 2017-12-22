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
		processThrottleController();
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
	TFrame_CPU_USAGE_STATS s;

	// Decimate the number of msgs sent to the PC:
	static uint8_t decim0 = 0;
	if (++decim0>100)
	{
		decim0=0;
		
	// Accumulate stats:
	s.payload.loop_average_time+=dt;
	if (dt>s.payload.loop_max_time) s.payload.loop_max_time=dt;
	if (dt<s.payload.loop_min_time) s.payload.loop_min_time=dt;

	// Send to main PC?
	const int CPU_STATS_DECIMATE = 0x1000;
	s.payload.loop_average_time /= CPU_STATS_DECIMATE;
	s.payload.timestamp_ms_tenths = millis();
	s.calc_and_update_checksum();
	s.payload.clear();

	UART::Write((uint8_t*)&s,sizeof(s));
	}
}
