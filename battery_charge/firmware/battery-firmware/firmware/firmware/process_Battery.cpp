/*
 * process_Battery.cpp
 *
 * Created: 05/10/2017 9:25:36
 *  Author: Francisco Jose Manas
 */ 
#include "libclaraquino/gpio.h"
#include "libclaraquino/modules/adc_ad7606/ad7606.h"
#include "libclaraquino/uart.h"
#include "batterycharge_declarations.h"
#include <stdio.h>

uint32_t	PC_last_millis = 0;
uint16_t	PC_sampling_period_ms_tenths = 5000;

 // TODO: Define an xxxx_init();
void process_batery_init()
{
	ad7606_config_t cfg;
	cfg.PIN_1stDATA = 0x23;
	cfg.PIN_BUSY	= 0x24;
	cfg.PIN_CONVST	= 0x27;
	cfg.PIN_DATA[0]	= 0x22;
	cfg.PIN_DATA[1]	= 0x21;
	cfg.PIN_DATA[2]	= 0x20;
	cfg.PIN_DATA[3]	= 0x10;
	cfg.PIN_DATA[4]	= 0x11;
	cfg.PIN_DATA[5]	= 0x12;
	cfg.PIN_DATA[6]	= 0x13;
	cfg.PIN_DATA[7]	= 0x14;
	cfg.PIN_nRD		= 0x25;
	cfg.PIN_RESET	= 0x26;
	
	mod_ad7606_init(cfg);
}

void processBattery()
{
	const uint32_t tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms_tenths)
	return;

	PC_last_millis = tnow;
	
	TFrame_BATTERY_readings tx;
	//----
	mod_ad7606_convst();
	mod_ad7606_wait_busy();
	int16_t buf[8];
	mod_ad7606_read_all(buf);
	
	char str[100];
	sprintf(str,"V0=%i V1=%i\r\n", buf[0],buf[1]);
	UART::WriteString(str);
}