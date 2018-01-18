/* libclaraquino (C) Copyright 2016-2018 University of Almeria
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "vehicle_controller2pc-structs.h"
#include <libclaraquino/mod_quad_encoder.h>

#include "libclaraquino/claraquino_config.h"
#include "libclaraquino/gpio.h"
#include "libclaraquino/millis_timer.h"
#include "libclaraquino/uart.h"
#include <avr/interrupt.h>  // cli()/sei()

uint32_t	PC_last_millis = 0;
uint16_t	PC_sampling_period_ms_tenths = 5000;
bool		ENCODERS_active       = false;

TFrame_ENCODERS_readings_payload_t enc_last_reading;


void init_encoders(const TFrameCMD_ENCODERS_start_payload_t &cmd)
{
	PC_sampling_period_ms_tenths = cmd.sampling_period_ms_tenths;

	// For each software-based encoder:
	for (uint8_t i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
	{
		mod_quad_encoder_init(i, cmd.encA_pin[i], cmd.encB_pin[i], cmd.encZ_pin[i]);
	}
	ENCODERS_active=true;
}

void processEncoders()
{
	if (!ENCODERS_active)
		return;

	const uint32_t tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms_tenths)
	return;

	PC_last_millis = tnow;

	TFrame_ENCODERS_readings tx;

	// Atomic read: used to avoid race condition while reading if an interrupt modified the mid-read data.
	uint8_t oldSREG = SREG;
	cli();
	for (uint8_t i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
	{
		tx.payload.encoders[i] = ENC_STATUS[i].COUNTER;
	}
	SREG=oldSREG;

	// send answer back:
	tx.payload.timestamp_ms_tenths = millis();
	tx.payload.period_ms_tenths = PC_sampling_period_ms_tenths;

	// Decimate the number of msgs sent to the PC:
	static uint8_t decim = 0;
	if (++decim>10)
	{
		decim=0;
		tx.calc_and_update_checksum();
		UART::Write((uint8_t*)&tx,sizeof(tx));
	}

	enc_last_reading = tx.payload;
}
