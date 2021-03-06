﻿/*
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

#include "vehicle_cruise_control2pc-structs.h"

#include <stdint.h>  // uint8_t, etc.

// Function prototypes:
void reset_rx_buf();
void processIncommingPkts();
void processADCs();          // 160us per ADC channel
void adc_process_start_cmd(const TFrameCMD_ADC_start_payload_t &adc_req);
void adc_process_stop_cmd();
void processEncoders();
void init_encoders(const TFrameCMD_ENCODERS_start_payload_t &cmd);
void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data);
void process_timeouts();
void flash_led(int ntimes, int nms);
void send_simple_opcode_frame(const uint8_t op);

// Throttle Controller Function:
void processThrottleController();
void enableThrottleController(bool enabled);
void setThrottle_ControllerParams(const TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t &throttle_controller_params);
void setOpenLoopSetpoint_VehVel(float ol_vel_mps);
void setControllerSetpoint_VehVel(float vel_mps);

// Brake Controller Function:
void processBrakeController();
void enableBrakeController(bool enable);
void setBrake_ControllerParams(const TFrameCMD_CONTROL_BRAKE_SET_PARAMS_payload_t &brake_controller_params);
void setOpenLoopSetpoint_Brake(int16_t ol_brakeforce);
void setControllerSetpoint_Brake(float brakeforce);

// Verbosity Control:
void setVerbosityControl(TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control);

void initSensorsForController();

// Global vars:
extern TFrame_ENCODERS_readings_payload_t enc_last_reading;
extern TFrameCMD_VERBOSITY_CONTROL_payload_t global_decimate;
extern TFrame_ADC_readings_payload_t ADC_last_reading;
