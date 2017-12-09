/*
 * process_steer_controller.cpp
 *
 * Created: 30/10/2017 22:59:28
 *  Author: Francisco Jose Ma�as Alvarez, Jose Luis Blanco Claraco
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
	#warning OPEN_LOOP!
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
	/** %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
	/*	Encoder reading and Smith predictor implementation*/
	rpm = (m_Encoder[0] - m_Encoder[1]) / T;
	m_ys[0] = m_ys[1] * 0.1709 - 0.973 * m_us[1+3];

	// Manual mode
	if (Manual_Control)
	{
		/* PWM */
		m_us[0] = round(Eje_x * 254);
		/*	Protection to detect the limit of mechanism */
		if (std::abs(m_Encoder[0]) >= max_p)
			lim = 1;
		if (std::abs(m_Encoder[0]) <= (max_p - 5) && lim == 1)
			lim = 0;
		if (lim == 1)
		{
			if(m_Encoder[0] > 0 && m_us[0] > 0)
				m_us[0] = 0;
			if(m_Encoder[0] < 0 && m_us[0] < 0)
				m_us[0] = 0;
		}
	}
	// Automatic mode
	else
	{
	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ */
	/*	Position reference reading */
		m_R_steer[0]	= (double)(Eje_x * 50);
	/*	Slope reference limit to overcurrent protection*/
		double pendiente = (m_R_steer[0] - m_R_steer[1]) / T;
		if (pendiente >= sat_ref)
			m_R_steer[0] = (m_R_steer[1] + sat_ref);
	/*	.............................*/
		m_R_steer[0] = - m_R_steer[0];
	/*	Position error. Extern loop*/
		m_ep[0] = m_R_steer[0] - m_Encoder[0];
	/*	Position controller */
		m_up[0] = m_up[1] + 11.142 * m_ep[0] - 19.9691 * m_ep[1] +8.8889 * m_ep[2];
	/*	Speed error. Intern loop*/
		m_es[0] = m_up[0] - m_ys[0] - (rpm - m_ys[3]);
	/*	Speed controller */
		m_us[0] = round(m_us[1] - 0.4542 * m_es[0] + 0.0281 * m_es[1]);
	/*	Variable to Anti-windup technique*/
		int m_v= m_us[0]; 
	/*	Protection to detect the limit of mechanism */
		if (std::abs(m_Encoder[0]) >= max_p)
		lim = 1;
		if (std::abs(m_Encoder[0]) <= (max_p - 5) && lim == 1)
		lim = 0;
		if (lim ==1)
		{
			if(m_Encoder[0] > 0 && m_us[0] > 0)
				m_us[0] = 0;
			if(m_Encoder[0] < 0 && m_us[0] < 0)
				m_us[0] = 0;
		}
	/*	Saturation */
		if (m_us[0] > 254)
			m_us[0] = 254;
		if (m_us[0] < -254)
			m_us[0] = -254;
	/*	Anti-windup technique*/
		if(m_us[0] - m_v != 0)
			m_antiwindup[0] = (m_us[0] - m_v) / sqrt(0.0283);
		else
			m_antiwindup[0] = 0;

		m_u[0] = round(0.5 * (2 * m_u[0] + 0.05 * (m_antiwindup[0] + m_antiwindup[1])));
	}

	/* Values actualization*/
	m_R_steer[1] = m_R_steer[0];
	m_u[1] = m_u[0];
	m_antiwindup[1] = m_antiwindup[0];
	for (int i=2;i>=1;i--)
	{
		m_yp[i] = m_yp[i-1];
		m_ep[i] = m_ep[i-1];
	}
	for (int i=5;i>=1;i--)
		m_up[i] = m_up[i-1];

	m_es[1] = m_es[0];
	m_Encoder[1] = m_Encoder[0];
	for (int i=3;i>=1;i--)
		m_ys[i] = m_ys[i-1];

	for (int i=4;i>=1;i--)
		m_us[i] = m_us[i-1];


	/*	Direction*/
	bool u_steer_dir = false;
	if (m_us[0] < 0)
	u_steer_dir = false;
	else
	u_steer_dir = true;

	uint8_t u_steer = abs(m_us[0]);

	// Output PWM:
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,u_steer);
	// PWM dir:
	gpio_pin_write(PWM_DIR, u_steer_dir);

	// (ii) CONTROL FOR MAIN VEHICLE MOTOR
	// -------------------------------------------------------------
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
		uint16_t veh_speed_dac = 1.0 + std::abs(Eje_y) * 4.76;
		// Output direction:
		if (Eje_y<0)
			gpio_pin_write(RELAY_FRWD_REV,true);
		else
			gpio_pin_write(RELAY_FRWD_REV,false);

		// Output value:
		mod_dac_max5500_update_single_DAC(0 /*DAC idx*/, veh_speed_dac);


}

