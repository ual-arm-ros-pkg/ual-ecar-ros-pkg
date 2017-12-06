/*
 * process_steer_controller.cpp
 *
 * Created: 30/10/2017 22:59:28
 *  Author: Francisco Jose Mañas Alvarez, Jose Luis Blanco Claraco
 *
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
#include "mod_ems22a.h"
#include "mod_dac_max5500.h"
#include "common/millis_timer.h"
#include "common/uart.h"
#include "common/pwm.h"
#include "common/gpio.h"

// ============== HARDWARE CONFIGURATION =====================
const uint16_t SAMPLING_PERIOD_MSth = 10 /*ms*/ *10 /*th*/;
const int8_t ENCODER_ABS_CS         = 0x20;
const int8_t ENCODER_ABS_CLK        = 0x21;
const int8_t ENCODER_ABS_DO         = 0x22;
const int8_t ENCODER_DIFF_A         = 0x42; // PD2
const int8_t ENCODER_DIFF_B         = 0x43; // PD3
const int8_t CURRENT_SENSE_ADC_CH   = 0;    // ADC #0
const int8_t PWM_DIR                = 0x47; // CW/CCW
const int8_t RELAY_FRWD_REV         = 0x11;
#define PWM_OUT_TIMER               PWM_TIMER2   // PD6=OC2B
#define PWM_OUT_PIN                 PWM_PIN_OCnB
const int8_t PWM_PIN_NO             = 0x46;
const int8_t DAC_OUT_PIN_NO         = 0x24; // PB4
// ===========================================================

uint32_t  CONTROL_last_millis = 0;
uint16_t  CONTROL_sampling_period_ms_tenths = 50 /*ms*/ * 10;
bool      STEERCONTROL_active = false;// true: controller; false: open loop

float Q_STEER_INT[3] = { -2.85f, -0.1765f, .0f };
float Q_STEER_EXT[3] = { 1.8903f, - 1.8240f, .0f };

/** Desired setpoint for steering angle. 
  * -512:max right, +511: max left
  */
int16_t  SETPOINT_STEER_POS = 0;

/** Time of when the setpoint was last changed (1/10 of ms) */
uint32_t SETPOINT_STEER_TIMESTAMP = 0;

/** Desired steering duration (in seconds)
  */
float   SETPOINT_STEER_TIME = 1.0f;


/** Start reading all required sensors at the desired rate */
void initSensorsForController()
{
	// Init encoder decoder:
	{
		TFrameCMD_ENCODERS_start_payload_t cmd;
		cmd.encA_pin[0] = ENCODER_DIFF_A;
		cmd.encB_pin[0] = ENCODER_DIFF_B;
		cmd.sampling_period_ms_tenths = SAMPLING_PERIOD_MSth;
		init_encoders(cmd);
	}

	// ADC: current sense of steering motor, to ADC0 pin
	{
		TFrameCMD_ADC_start_payload_t cmd;
		cmd.active_channels[0] = CURRENT_SENSE_ADC_CH;
		cmd.use_internal_refvolt = false;
		cmd.measure_period_ms_tenths = SAMPLING_PERIOD_MSth;
		adc_process_start_cmd(cmd);
	}

	// ABS ENC: 
	{
		init_EMS22A(ENCODER_ABS_CS,ENCODER_ABS_CLK,ENCODER_ABS_DO,SAMPLING_PERIOD_MSth);
		EMS22A_active = true;
	}
	
	// Init DAC:
	mod_dac_max5500_init();

	// PWM:
	gpio_pin_mode(PWM_PIN_NO, OUTPUT);
	pwm_init(PWM_OUT_TIMER, PWM_PRESCALER_1 );
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,0x00);
	// PWM dir:	
	gpio_pin_mode(PWM_DIR, OUTPUT);
	gpio_pin_write(PWM_DIR, false);

	// Relay:
	gpio_pin_mode(RELAY_FRWD_REV, OUTPUT);
	gpio_pin_write(RELAY_FRWD_REV, false);
	
	// DAC:
	PIN_DAC_MAX5500_CS = DAC_OUT_PIN_NO;
	
}

// TODO: "a" constant that converts from diff encoder tick count to abs enc tick count

void enableSteerController(bool enabled)
{
	STEERCONTROL_active = enabled;
	
	#warning Set PWM & DAC values to safe values in any case.
}

void setSteer_SteeringParams(const TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t &p)
{
	for (int i=0;i<3;i++)
	{
		Q_STEER_INT[i] = p.Q_STEER_INT[i];
		Q_STEER_EXT[i] = p.Q_STEER_EXT[i];
	}
	// Anything else??
}

void setSteerControllerSetpoint_Steer(int16_t pos, float dtime)
{
	SETPOINT_STEER_POS=pos;
	SETPOINT_STEER_TIME=dtime;
	SETPOINT_STEER_TIMESTAMP = millis();
}
void setSteerControllerSetpoint_VehVel(float vel_mps)
{
	#warning Write me!
}


void processSteerController()
{
	if (!STEERCONTROL_active)
		return;

	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis < CONTROL_sampling_period_ms_tenths)
	return;
	CONTROL_last_millis = tnow;

	// ========= Control algorithm for: (i) steering, (ii) vehicle main motor =====

	// (i) CONTROL FOR STEERING WHEEL
	// -------------------------------------------------------------

	// last differential encoder:
	int32_t enc_diff = enc_last_reading.encoders[0];

	// Magic here!


	uint8_t u_steer  = 0x00; // [0,255]
	bool u_steer_dir = false;

	// Output GPIO cw/ccw rotation direction:
	//gpio_pin_write();

	// Output PWM:
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,u_steer);
	// PWM dir:
	gpio_pin_write(PWM_DIR, u_steer_dir);

	// (ii) CONTROL FOR MAIN VEHICLE MOTOR
	// -------------------------------------------------------------

		
	uint16_t veh_speed_dac = 0; // TODO

	// Output value:
	mod_dac_max5500_update_single_DAC(0 /*DAC idx*/, veh_speed_dac);
	
	// Output direction:
	gpio_pin_write(RELAY_FRWD_REV,false);

}

