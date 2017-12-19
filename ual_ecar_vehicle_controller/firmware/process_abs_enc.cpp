/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-17, Universidad de Almeria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "steer_controller2pc-structs.h"
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
	if (++decim0>10)
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

