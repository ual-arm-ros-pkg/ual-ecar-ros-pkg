/*
 * process_controller.cpp
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
#include "libclaraquino/mod_ems22a.h"
#include "libclaraquino/mod_dac_max5500.h"
#include "libclaraquino/millis_timer.h"
#include "libclaraquino/uart.h"
#include "libclaraquino/pwm.h"
#include "libclaraquino/gpio.h"
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
float	Ys[4]			=	{0,0,0,0};		// Smith predictor output
float	T				=	0.01f;			// Sample time
float	Encoder_dir[2]	=	{0,0};			// Direction value
float	U_control[6]	=	{0,0,0,0,0,0};	// Control signal
float	Ref_pos[2]		=	{0,0};			// Position reference
float	Ref_speed[2]	=	{0,0};			// Speed reference
float	Error_pos[3]	=	{0,0,0};		// Position error
float	Error_speed[3]	=	{0,0,0};		// Speed error
float	Antiwindup[2]	=	{0,0};			//

// Auxiliary vars:
bool	lim				=	false;
float	max_p			=	500;
const float	sat_ref			=	100;
float	enc_init		=	.0f;
static	uint8_t adjust	=	0;
float	pedal			=	.0f; /* [0,1] */
const float ANTIWINDUP_CTE = sqrt(0.0283);

uint32_t  CONTROL_last_millis_STEER = 0;
uint32_t  CONTROL_last_millis_THROTTLE = 0;
uint16_t  CONTROL_sampling_period_ms_tenths = 50 /*ms*/ * 10;
bool      STEERCONTROL_active = false;// true: controller; false: open loop
bool	  THROTTLECONTROL_active = false; //true: controller; false: open loop

float	Q_STEER_INT[3]				= { - 0.2838f, 0.1986f, .0f };
float	Q_THROTTLE[3]				= {0,0,0};
float	Q_STEER_EXT[3]				= { 46.6728f, - -91.1049f, 44.4444f };
float	P_SMITH_SPEED[5]			= {0.2977f,.0f,1,-0.7023f,.0f}; /*{b0,b1,a0,a1,a2}*/
uint8_t	U_STEER_FEEDFORWARD[2]		= {0,0}; /*Weight,other*/
uint8_t	U_STEER_DECOUPLING[2]		= {0,0}; /*battery-charge,speed*/
uint8_t	U_THROTTLE_FEEDFORWARD[2]	= {0,0}; /*Weight,other*/	
uint8_t U_THROTTLE_DECOUPLING		= 0; /*battery-charge*/

/** Desired setpoint for steering angle. 
  * -512:max right, +511: max left
  */
int16_t  SETPOINT_STEER_POS = 0;

/** Desired setpoint for steering angle in Open Loop. 
  * -254:max right, +254: max left
  */
uint16_t SETPOINT_OPENLOOP_STEER_SPEED = 0;

/** Desired setpoint for throttle in Open Loop. 
  * [-1,0]V:max reverse, [0,+1]V: max forward
  */
float SETPOINT_OPENLOOP_THROTTLE = .0f;

/** Desired setpoint for throttle in Open Loop. 
  * 0:min speed, 12.5 m/s: max forward
  */
float SETPOINT_CONTROL_THROTTLE_SPEED = .0f;

/** Time of when the setpoint was last changed (1/10 of ms) */
uint32_t SETPOINT_STEER_TIMESTAMP = 0;
uint32_t SETPOINT_OPENLOOP_STEER_TIMESTAMP = 0;
uint32_t SETPOINT_OPENLOOP_THROTTLE_TIMESTAMP = 0;
uint32_t SETPOINT_CONTROL_THROTTLE_SPEED_TIMESTAMP = 0;

template <typename T, size_t N>
void do_shift(T (&v)[N])
{
	for (int i=N-1;i>=1;i--)
		v[i] = v[i-1];
}


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
}

void enableThrottleController(bool enabled)
{
	THROTTLECONTROL_active = enabled;
}

void setSteer_ControllerParams(const TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t &p)
{
	for (int i=0;i<3;i++)
	{
		Q_STEER_INT[i] = p.Q_STEER_INT[i];
		Q_STEER_EXT[i] = p.Q_STEER_EXT[i];
	}
	for (int i=0;i<5;i++)
		P_SMITH_SPEED[i] = p.P_SMITH_SPEED[i];
		
	for (int i=0;i<2;i++)
	{
		U_STEER_DECOUPLING[i]	= p.U_STEER_DECOUPLING[i];
		U_STEER_FEEDFORWARD[i]	= p.U_STEER_FEEDFORWARD[i];
	}
}
void setThrottle_ControllerParams(const TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t &p)
{
	for (int i=0;i<3;i++)
		Q_THROTTLE[i] = p.Q_THROTTLE_CONTROLLER[i];
		
	for (int i=0;i<2;i++)
		U_THROTTLE_FEEDFORWARD[i] = p.U_THROTTLE_FEEDFORWARD[i];
		
	U_THROTTLE_DECOUPLING = p.U_THROTTLE_DECOUPLING;
}

void setSteerOpenLoopSetpoint_Steer(int16_t speed)
{
	SETPOINT_OPENLOOP_STEER_SPEED = speed;
	SETPOINT_OPENLOOP_STEER_TIMESTAMP = millis();
}
void setSteerControllerSetpoint_Steer(int16_t pos)
{
	SETPOINT_STEER_POS=pos;
	SETPOINT_STEER_TIMESTAMP = millis();
}
void setSteerOpenLoopSetpoint_VehVel(float ol_vel_mps)
{
	SETPOINT_OPENLOOP_THROTTLE = ol_vel_mps;
	SETPOINT_OPENLOOP_THROTTLE_TIMESTAMP = millis();
}
void setSteerControllerSetpoint_VehVel(float vel_mps)
{
	SETPOINT_CONTROL_THROTTLE_SPEED = vel_mps;
	SETPOINT_CONTROL_THROTTLE_SPEED_TIMESTAMP = millis();
}

// Stopwatch: 0.35 ms
void processSteerController()
{
	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis_STEER < CONTROL_sampling_period_ms_tenths)
		return;
		
	CONTROL_last_millis_STEER = tnow;
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
		U_control[0] = SETPOINT_OPENLOOP_STEER_SPEED;
		/*	Protection to detect the limit of mechanism */
		if (abs(Encoder_dir[0]) >= max_p)
			lim = true;
		if (abs(Encoder_dir[0]) <= (max_p - 5) && lim)
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
		Ref_pos[0]	= SETPOINT_STEER_POS;
	/*	Slope reference limit to over current protection*/
		float pendiente = (Ref_pos[0] - Ref_pos[1]) / T;
		if (pendiente >= sat_ref)
			Ref_pos[0] = (Ref_pos[1] + sat_ref);
	/*	Correction of the wheels' direction */
		Ref_pos[0] = - Ref_pos[0];
	/*	Position error. Extern loop*/
		Error_pos[0] = Ref_pos[0] - Encoder_dir[0];
	/*	Position controller */
		Ref_speed[0] = Ref_speed[1] + Q_STEER_EXT[0] * Error_pos[0] + Q_STEER_EXT[1] * Error_pos[1] + Q_STEER_EXT[2] * Error_pos[2];
	/*	Speed error. Intern loop*/
		Error_speed[0] = Ref_speed[0] - Ys[0] - (rpm - Ys[3]);
	/*	Speed controller */
		U_control[0] = U_control[1] + Q_STEER_INT[0] * Error_speed[0] + Q_STEER_INT[1] * Error_speed[1] + Q_STEER_INT[2] * Error_speed[2];
	/*	Control Signal with feedforward & decoupling*/
		U_control[0] = U_control[0] + U_STEER_DECOUPLING[0] + U_STEER_DECOUPLING[1] + U_STEER_FEEDFORWARD[0] + U_STEER_FEEDFORWARD[1];
	/*	Variable to Anti-windup technique*/
		int m_v= round(U_control[0]);
	/*	Saturation */
		bool has_sat = false;
		if (U_control[0] > 254)
		{
			U_control[0] = 254;
			has_sat = true;
		}
		if (U_control[0] < -254)
		{
			U_control[0] = -254;
			has_sat = true;
		}
	/*	Anti-windup technique*/
		if(has_sat)
			Antiwindup[0] = (U_control[0] - m_v) / ANTIWINDUP_CTE;
		else
			Antiwindup[0] = 0;

		U_control[0] = round(0.5 * (2 * U_control[0] + T * (Antiwindup[0] + Antiwindup[1])));
	} // end automatic control

	/* Values actualization*/
	do_shift(Ref_pos);
	do_shift(Antiwindup);
	do_shift(Error_pos);
	do_shift(Ref_speed);
	do_shift(Error_speed);
	do_shift(Encoder_dir);
	do_shift(Ys);
	do_shift(U_control);

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

}
void processThrottleController()
{
	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis_THROTTLE < CONTROL_sampling_period_ms_tenths)
		return;

	CONTROL_last_millis_THROTTLE = tnow;

		// (ii) CONTROL FOR MAIN VEHICLE MOTOR
	// -------------------------------------------------------------
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
	if (!THROTTLECONTROL_active)
		pedal = SETPOINT_OPENLOOP_THROTTLE;
	else
	{
		// Throttle-by-wire controller here!!
		pedal = SETPOINT_CONTROL_THROTTLE_SPEED / 12.5;
	}
	// Output direction:
	if (pedal<0)
		gpio_pin_write(RELAY_FRWD_REV,true);
	else
		gpio_pin_write(RELAY_FRWD_REV,false);

	// Output value:
	uint16_t veh_speed_dac = 1.0 + abs(pedal) * 4; /* 1.0 : Offset */
	mod_dac_max5500_update_single_DAC(0 /*DAC idx*/, veh_speed_dac);

}

