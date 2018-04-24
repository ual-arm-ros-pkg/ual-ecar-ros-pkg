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
#include "libclaraquino/millis_timer.h"
#include <stdio.h>
#include <avr/interrupt.h>  // cli()/sei()
#include <libclaraquino/delays.h> // delay

uint32_t	PC_last_millis = 0;
uint16_t	PC_sampling_period_ms_tenths = 2000;
//TFrameCMD_VERBOSITY_CONTROL_payload_t global_decimate;

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

// void setVerbosityControl(TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control)
// {
// 	global_decimate = verbosity_control;
// }

void processBattery()
{
	const uint32_t tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms_tenths)
	return;
	PC_last_millis = tnow;
	
	TFrame_BATTERY_readings tx;
	
	// Atomic read: used to avoid race condition while reading if an interrupt modified the mid-read data.
	uint8_t oldSREG = SREG;
	cli();
	mod_ad7606_convst();
	mod_ad7606_wait_busy();
	int16_t buf[8];
	mod_ad7606_read_all(buf);
	for (uint8_t i=0;i<8;i++)
	{
		tx.payload.bat_volts[i] = buf[i];
	}
	#warning Cambiar
	tx.payload.bat_current = 0;
	SREG=oldSREG;

	sei();
	
	char str[100];
	sprintf(str,"V0=%i V1=%i V2=%i V3=%i V4=%i V5=%i V6=%i V7=%i \r\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	UART::WriteString(str);

	// send answer back:
	tx.payload.timestamp_ms_tenths = millis();
	tx.payload.period_ms_tenths = PC_sampling_period_ms_tenths;
/*
	// Decimate the number of msgs sent to the PC:
	static uint8_t decim = 0;
	if (++decim>10)
	{
		decim=0;
		tx.calc_and_update_checksum();
		UART::Write((uint8_t*)&tx,sizeof(tx));
	}
	*/
}