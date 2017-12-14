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
#include "math.h"

// ============== HARDWARE CONFIGURATION =====================
const uint16_t SAMPLING_PERIOD_MSth = 10 /*ms*/ *10 /*th*/;
const int8_t ENCODER_ABS_CS         = 0x20; //PB0
const int8_t ENCODER_ABS_CLK        = 0x21; //PB1
const int8_t ENCODER_ABS_DO         = 0x22; //PB2
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
// Control vars:
double	Ys[4]			=	{0,0,0,0};		// Smith predictor output
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
double	enc_init		=	0;
static uint8_t adjust	=	0;

uint32_t  CONTROL_last_millis = 0;
uint16_t  CONTROL_sampling_period_ms_tenths = 50 /*ms*/ * 10;
bool      STEERCONTROL_active = false;// true: controller; false: open loop

float Q_STEER_INT[3]	= { - 0.4542f, 0.0281f, .0f };
float Q_STEER_EXT[3]	= { 11.142f, - 19.9691f, 8.8889f };
float P_SMITH_SPEED[5]	= {0.8291,0,1,-0.1709,0}; /*{b0,b1,a0,a1,a2}*/
int16_t Axis[2]		= {0,0}; /* {Eje_x:steer [-255,+255], Eje_y}*/

/** Desired setpoint for steering angle. 
  * -512:max right, +511: max left
  */
int16_t  SETPOINT_STEER_POS = 0;

/** Time of when the setpoint was last changed (1/10 of ms) */
uint32_t SETPOINT_STEER_TIMESTAMP = 0;

/** Desired setpoint for vehicle speed.
  * :min_speed , :max_speed
  */


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
	// PWM direction:	
	gpio_pin_mode(PWM_DIR, OUTPUT);
	gpio_pin_write(PWM_DIR, false);

	// Relay:
	gpio_pin_mode(RELAY_FRWD_REV, OUTPUT);
	gpio_pin_write(RELAY_FRWD_REV, false);
	
	// DAC:
	PIN_DAC_MAX5500_CS = DAC_OUT_PIN_NO;
	
}

// TODO: "a" constant that converts from diff encoder tick count to abs enc tick count :: 337/(500*100);

void enableSteerController(bool enabled)
{
	STEERCONTROL_active = enabled;
	
	// setJoystickValue =>	#warning Set PWM & DAC values to safe values in any case.
}

void setSteer_SteeringParams(const TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t &p)
{
	for (int i=0;i<3;i++)
	{
		Q_STEER_INT[i] = p.Q_STEER_INT[i];
		Q_STEER_EXT[i] = p.Q_STEER_EXT[i];
	}
	for (int i=0;i<2;i++)
		P_SMITH_SPEED[i] = p.P_SMITH_SPEED[i];

}
void setJoystickValue(const TFrameCMD_JOYSTICK_VALUE_payload_t &j)
{
	for (int i=0;i<2;i++)
		Axis[i]=j.Axis[i];
}

void setSteerControllerSetpoint_Steer(int16_t pos)
{
	SETPOINT_STEER_POS=pos;
	SETPOINT_STEER_TIMESTAMP = millis();
}
void setSteerControllerSetpoint_VehVel(float vel_mps)
{
	#warning Write me!
}


void processSteerController()
{
	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis < CONTROL_sampling_period_ms_tenths)
	CONTROL_last_millis = tnow;
	// ========= Encoders calibration algorithm ===================================
	// Incremental encoder reading
	int32_t enc_diff = enc_last_reading.encoders[0];
	// Absolute encoder reading
	// int16_t enc_abs = enc_abs_last_reading.enc_pos;
	// Calibration
	if (++adjust>200)
	{
		adjust = 0;
		enc_init = enc_abs_last_reading.enc_pos - enc_last_reading.encoders[0] * 0.0067;
	}
	Encoder_dir[0] = enc_init + enc_diff * 0.0067; /* *0.0067 = *337 / (500 * 100); */
	// ========= Control algorithm for: (i) steering, (ii) vehicle main motor =====

	// (i) CONTROL FOR STEERING WHEEL
	// -------------------------------------------------------------
	/*	Encoder reading and Smith Predictor implementation*/
	float rpm = (Encoder_dir[0] - Encoder_dir[1]) / T;
	Ys[0] = (- Ys[1] * P_SMITH_SPEED[3] - Ys[2] * P_SMITH_SPEED[4] + P_SMITH_SPEED[0] * U_control[1+3] + P_SMITH_SPEED[1] * U_control[2+3])/P_SMITH_SPEED[2];

	// Manual mode
	if (!STEERCONTROL_active)
	{
		/* PWM */
		U_control[0] = round(Axis[0] * 254);
		/*	Protection to detect the limit of mechanism */
		if (abs(Encoder_dir[0]) >= max_p)
			lim = true;
		if (abs(Encoder_dir[0]) <= (max_p - 10) && lim)
			lim = false;
		if (lim)
		{
			if(Encoder_dir[0] > 0 && U_control[0] > 0)
				U_control[0] = 0;
			if(Encoder_dir[0] < 0 && U_control[0] < 0)
				U_control[0] = 0;
		}
	}
	// Automatic mode
	else
	{
	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ */
	/*	Position reference reading */
		Ref_pos[0]	= (double)(Axis[0] * 50);
	/*	Slope reference limit to over current protection*/
		double pendiente = (Ref_pos[0] - Ref_pos[1]) / T;
		if (pendiente >= sat_ref)
			Ref_pos[0] = (Ref_pos[1] + sat_ref);
	/*	.............................*/
		Ref_pos[0] = - Ref_pos[0];
	/*	Position error. Extern loop*/
		Error_pos[0] = Ref_pos[0] - Encoder_dir[0];
	/*	Position controller */
		Ref_speed[0] = Ref_speed[1] + Q_STEER_EXT[0] * Error_pos[0] + Q_STEER_EXT[1] * Error_pos[1] + Q_STEER_EXT[2] * Error_pos[2];
	/*	Speed error. Intern loop*/
		Error_speed[0] = Ref_speed[0] - Ys[0] - (rpm - Ys[3]);
	/*	Speed controller */
		U_control[0] = round(U_control[1] + Q_STEER_INT[0] * Error_speed[0] + Q_STEER_INT[1] * Error_speed[1] + Q_STEER_INT[2] * Error_speed[2]);
	/*	Variable to Anti-windup technique*/
		int m_v= U_control[0]; 
	/*	Protection to detect the limit of mechanism */
		if (abs(Encoder_dir[0]) >= max_p)
		lim = 1;
		if (abs(Encoder_dir[0]) <= (max_p - 5) && lim == 1)
		lim = 0;
		if (lim ==1)
		{
			if(Encoder_dir[0] > 0 && U_control[0] > 0)
				U_control[0] = 0;
			if(Encoder_dir[0] < 0 && U_control[0] < 0)
				U_control[0] = 0;
		}
	/*	Saturation */
		if (U_control[0] > 254)
			U_control[0] = 254;
		if (U_control[0] < -254)
			U_control[0] = -254;
	/*	Anti-windup technique*/
		if(U_control[0] - m_v != 0)
			Antiwindup[0] = (U_control[0] - m_v) / sqrt(0.0283);
		else
			Antiwindup[0] = 0;

		U_control[0] = round(0.5 * (2 * U_control[0] + 0.05 * (Antiwindup[0] + Antiwindup[1])));
	}

	/* Values actualization*/
	Ref_pos[1] = Ref_pos[0];
	Antiwindup[1] = Antiwindup[0];
	for (int i=2;i>=1;i--)
	{
		Error_pos[i] = Error_pos[i-1];
	}
	for (int i=5;i>=1;i--)
		Ref_speed[i] = Ref_speed[i-1];

	Error_speed[1] = Error_speed[0];
	Encoder_dir[1] = Encoder_dir[0];
	for (int i=3;i>=1;i--)
		Ys[i] = Ys[i-1];

	for (int i=5;i>=1;i--)
		U_control[i] = U_control[i-1];


	/*	Direction*/
	bool u_steer_dir = false;
	if (U_control[0] < 0)
		u_steer_dir = false;
	else
		u_steer_dir = true;

	uint8_t u_steer = abs(U_control[0]);

	#warning Check max current and stop?

	// Output PWM:
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,u_steer);
	// PWM direction:
	gpio_pin_write(PWM_DIR, u_steer_dir);

	// (ii) CONTROL FOR MAIN VEHICLE MOTOR
	// -------------------------------------------------------------
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
		uint16_t veh_speed_dac = 1.0 + abs(Axis[1]) * 4.76;
		// Output direction:
		if (Axis[1]<0)
			gpio_pin_write(RELAY_FRWD_REV,true);
		else
			gpio_pin_write(RELAY_FRWD_REV,false);

		// Output value:
		mod_dac_max5500_update_single_DAC(0 /*DAC idx*/, veh_speed_dac);

	return;
}

