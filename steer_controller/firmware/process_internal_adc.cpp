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

#include "steer_controller_declarations.h"
#include "steer_controller2pc-structs.h"
#include "common/uart.h"
#include "common/millis_timer.h"
#include "common/adc_internal.h"

// ADC reading subsystem:
uint8_t        num_active_ADC_channels = 0;
#define MAX_ADC_CHANNELS 8
uint8_t        ADC_active_channels[MAX_ADC_CHANNELS] = {0,0,0,0,0,0,0,0};
uint32_t  ADC_last_millis = 0;
uint16_t       ADC_sampling_period_ms_tenths = 2000;

void adc_process_start_cmd(const TFrameCMD_ADC_start_payload_t &adc_req)
{
	// Setup vars for ADC task:
	num_active_ADC_channels = 0;
	for (int i=0;i<MAX_ADC_CHANNELS;i++) {
		ADC_active_channels[i] = 0;
		if (adc_req.active_channels[i]>=0) {
			ADC_active_channels[i] = adc_req.active_channels[i];
			num_active_ADC_channels++;
		}
	}
	ADC_sampling_period_ms_tenths = adc_req.measure_period_ms_tenths;
	
	// Enable ADC with internal/default reference:
	if (num_active_ADC_channels)
		adc_init(adc_req.use_internal_refvolt);
}

void adc_process_stop_cmd()
{
	num_active_ADC_channels = 0;
}



void processADCs()
{
	if (!num_active_ADC_channels)
		return;

	const uint32_t tnow = millis();

	if (tnow-ADC_last_millis < ADC_sampling_period_ms_tenths)
		return;

	ADC_last_millis = tnow;

	TFrame_ADC_readings tx;
	for (int i=0;i<MAX_ADC_CHANNELS;i++) {
		tx.payload.adc_data[i] = 0;
	}
	
	for (uint8_t i=0;i<num_active_ADC_channels;i++)
	{
		tx.payload.adc_data[i] = adc_read(ADC_active_channels[i]);
	}

	#warning "TODO: Decimate UART! & save last value"
	// send answer back:
	tx.payload.timestamp_ms_tenths = millis();
	tx.calc_and_update_checksum();

	UART::Write((uint8_t*)&tx,sizeof(tx));
}
