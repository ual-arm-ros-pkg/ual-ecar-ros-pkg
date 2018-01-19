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
#include "vehicle_controller_declarations.h"
#include <libclaraquino/mod_ems22a.h>
#include <libclaraquino/uart.h>
#include <libclaraquino/millis_timer.h>

uint32_t  EMS22A_last_millis        = 0;
extern uint16_t  EMS22A_sampling_period_ms_tenths;  // libclaraquino
bool      EMS22A_active             = false;
TFrame_ENCODER_ABS_reading_payload_t enc_abs_last_reading;


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
	// Decimate the number of msgs sent to the PC:
	static uint8_t decim0 = 0;
	if (++decim0>global_decimate.decimate_ENCABS)
	{
		decim0=0;
		
		tx.payload.timestamp_ms_tenths = tnow;
		tx.payload.enc_pos = enc_pos;
		tx.payload.enc_status = enc_status;
		tx.calc_and_update_checksum();

		UART::Write((uint8_t*)&tx,sizeof(tx));
	}
	enc_abs_last_reading = tx.payload;
}

