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

#pragma once

#include "steer_controller2pc-structs.h"

#include <stdint.h>  // uint8_t, etc.

// Function prototypes:
void reset_rx_buf();
void processIncommingPkts();
void processADCs();          // 160us per ADC channel
void adc_process_start_cmd(const TFrameCMD_ADC_start_payload_t &adc_req);
void adc_process_stop_cmd();
void processEncoders();
void init_encoders(const TFrameCMD_ENCODERS_start_payload_t &cmd);
void processEMS22A();        // 193us (Real: 300-400 us)
void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data);
void process_timeouts();
void flash_led(int ntimes, int nms);
void send_simple_opcode_frame(const uint8_t op);

void processSteerController();
void enableSteerController(bool enabled);
void setSteer_SteeringParams(const TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t &p);
void setSteerControllerSetpoint_Steer(int16_t pos, float dtime);
void setSteerControllerSetpoint_VehVel(float vel_mps);
void initSensorsForController();

// Global vars:
extern bool STEERCONTROL_active;

extern TFrame_ENCODERS_readings_payload_t enc_last_reading;

// Control vars:
double	Ys[4]			=	{0,0,0,0};		// Smith predictor output
float	rpm				=	0;				//
float	T				=	0.05;			// Sample time
double	Encoder_dir[2]	=	{0,0};			// Direction value
double	U_control[6]	=	{0,0,0,0,0,0};	// Control signal 
double	Ref_pos[2]		=	{0,0};			// Position reference
double	Ref_speed[2]	=	{0,0};			// Speed reference
double	Error_pos[3]	=	{0,0,0};		// Position error
double	Error_speed[3]	=	{0,0,0};		// Speed error
double	Antiwindup[2]	=	{0,0};			// 
		
// Auxiliary vars:
bool	lim				=	false;
float	max_p			=	500;
double	sat_ref			=	200;

// - 0.4542 + 0.0281