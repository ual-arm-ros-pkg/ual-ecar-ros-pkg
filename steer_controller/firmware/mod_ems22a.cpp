/*
 * process_EMS22A.cpp
 *
 * Created: 20/07/2017 10:13:46
 *  Author: Francisco José Mañas
 */ 

#include "steer_controller_declarations.h"
#include "steer_controller2pc-structs.h"
#include "mod_ems22a.h"
#include "common/delays.h"
#include "common/gpio.h"
#include "common/millis_timer.h"
#include "common/uart.h"

/* Absolute encoder EMS22A reading waveforms (from its datasheet) */
//    ____                                                                  ____
// Cs:    |                                                                |    |
//        |________________________________________________________________|    |_____
//    __________   _   _   _   _   _   _   _   _   _   _   _   _   ______________
// Clk:         | | | | | | | | | | | | | | | | | | | | | | | | | |
//              |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
//
//                  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _
// DO:__________   / \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \______________
//              |_|\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/
//                  D9-D8-D7-D6-D5-D4-D3-D2-D1-D0-S1-S2-S3-S4-S5-P1
//
// D0-D9: Absolute angular position data.
// S1	: End of offset compensation algorithm.
// S2	: Cordic overflow indicating an error in cordic part.
// S3	: Linearity alarm.
// S4	: Increase in magnitude.
// S5	: Decrease in magnitude.
// P1	: Even parity for detecting bits 1-15 transmission error.

int ENCODER_ABS_CS  = 0x10;
int ENCODER_ABS_CLK = 0x11;
int ENCODER_ABS_DO  = 0x12;
unsigned long  EMS22A_last_millis        = 0;
uint16_t       EMS22A_sampling_period_ms_tenths = 2000;
bool           EMS22A_active             = false;

bool init_EMS22A(int8_t init_ENCODER_ABS_CS, int8_t init_ENCODER_ABS_CLK, int8_t init_ENCODER_ABS_DO, uint16_t init_sampling_period_ms_tenths)
{
	ENCODER_ABS_CS  = init_ENCODER_ABS_CS;
	ENCODER_ABS_CLK = init_ENCODER_ABS_CLK;
	ENCODER_ABS_DO  = init_ENCODER_ABS_DO;
	EMS22A_sampling_period_ms_tenths = init_sampling_period_ms_tenths;
	if (ENCODER_ABS_CS<=0 || ENCODER_ABS_CLK<=0 || ENCODER_ABS_DO<=0)
		return false; // error
	
	gpio_pin_mode(ENCODER_ABS_CS, OUTPUT);
	gpio_pin_mode(ENCODER_ABS_CLK, OUTPUT);
	gpio_pin_mode(ENCODER_ABS_DO, INPUT);

	gpio_pin_write(ENCODER_ABS_CLK, true);
	gpio_pin_write(ENCODER_ABS_CS, true);
	
	return true; // all ok
}

uint16_t read_EMS22A()
{
	gpio_pin_mode(ENCODER_ABS_CS, OUTPUT);
	gpio_pin_mode(ENCODER_ABS_CLK, OUTPUT);
	gpio_pin_mode(ENCODER_ABS_DO, INPUT);

	gpio_pin_write(ENCODER_ABS_CS, false);
	delay_us(2);

	uint16_t pos = 0;
	for (int i=0; i<16; i++) {
		gpio_pin_write(ENCODER_ABS_CLK, false);  delay_us(1);
		gpio_pin_write(ENCODER_ABS_CLK, true); delay_us(1);

		pos = pos << 1; // shift 1 bit left
		if (gpio_pin_read(ENCODER_ABS_DO) == true )
		{
			pos |= 0x01;
		}
	}

	gpio_pin_write(ENCODER_ABS_CS, true);
	return pos;
}

void processEMS22A()
{
	if (!EMS22A_active)
	{
		return;
	}
	
	const uint32_t tnow = millis(); /* the current time ("now") */

	if (tnow-EMS22A_last_millis < EMS22A_sampling_period_ms_tenths)
	{
		return;
	}
	EMS22A_last_millis = tnow;

	const uint16_t dat = read_EMS22A();
	
	// Extract the position part and the status part
	const uint16_t enc_pos = dat >> 6;
	const uint8_t  enc_status = dat & 0x3f;
	
	TFrame_ENCODER_ABS_reading tx;
	// send answer back:
	tx.payload.timestamp_ms_tenths = tnow;
	tx.payload.enc_pos = enc_pos;
	tx.payload.enc_status = enc_status;
	tx.calc_and_update_checksum();

	UART::Write((uint8_t*)&tx,sizeof(tx));
}
