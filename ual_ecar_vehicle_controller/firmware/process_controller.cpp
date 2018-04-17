/*
 * process_controller.cpp
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

#include "vehicle_controller_steer_declarations.h"
#include "libclaraquino/mod_ems22a.h"
#include "libclaraquino/mod_dac_max5500.h"
#include "libclaraquino/millis_timer.h"
#include "libclaraquino/uart.h"
#include "libclaraquino/pwm.h"
#include "libclaraquino/gpio.h"
#include "math.h"
#include <avr/interrupt.h> // sei()

// ============== HARDWARE CONFIGURATION =====================
const uint16_t SAMPLING_PERIOD_MSth = 10 /*ms*/ *10 /*th*/;
const int8_t ENCODER_ABS_CS         = 0x20; //PB0
const int8_t ENCODER_ABS_CLK        = 0x21; //PB1
const int8_t ENCODER_ABS_DO         = 0x22; //PB2
const int8_t ENCODER_DIFF_A         = 0x42; // PD2
const int8_t ENCODER_DIFF_B         = 0x43; // PD3
const int8_t CURRENT_SENSE_ADC_CH   = 0;    // ADC #0
const int8_t PWM_DIR                = 0x44; // CW/CCW
#define PWM_OUT_TIMER               PWM_TIMER2   // PD6=OC2B
#define PWM_OUT_PIN                 PWM_PIN_OCnB
const int8_t PWM_PIN_NO             = 0x46; // PD6
// ===========================================================

const uint32_t WATCHDOG_TIMEOUT_msth = 1000*10;  // timeout for watchdog timer (both, openloop & controller)

// Control vars:
float	Ys[4]					=	{0,0,0,0};		// Smith predictor output
float	T						=	0.01f;			// Sample time
int16_t	Encoder_dir[2]			=	{0,0};			// Direction value
int16_t	U_steer_controller[6]	=	{0,0,0,0,0,0};	// Steer controller signal
float	Ref_pos[2]				=	{0,0};			// Position reference
float	Ref_speed[2]			=	{0,0};			// Speed reference
float	Error_pos[3]			=	{0,0,0};		// Position error
float	Error_speed[3]			=	{0,0,0};		// Speed error
float	Antiwindup[2]			=	{0,0};			// Antiwindup variable
TFrameCMD_VERBOSITY_CONTROL_payload_t global_decimate;

// Auxiliary vars:
bool	steer_mech_limit_reached= false;			// Enable security limit of the mechanism
#warning Cambiar limite
uint16_t Steer_offset			= 0;				// Steer offset regulated by software
int16_t	steer_mech_limit_pos	= Steer_offset + (1024*1.33-100)/2; // Safety limit of the mechanism. In units of absolute encoder.
																	// 100 = Safety margin
uint16_t abs_enc_pos			= 0;				// Init variable
const	float	sat_ref			= 250/T;			// Slope position reference limit to over current protection
int16_t	enc_offset_correction	= .0f;				// Incremental encoder calibration offset
float ANTIWINDUP_CTE = sqrt(0.0283);				// Antiwindup "constant" regulated by software

uint32_t  CONTROL_last_millis_STEER = 0;			// Timer to check the sample time
uint16_t  CONTROL_sampling_period_ms_tenths = 50 /*ms*/ * 10;
bool      STEERCONTROL_active = false;				// true: controller; false: open loop

float	Q_STEER_INT[3]				= { - 0.2838f, 0.1986f, .0f };	/* Steer speed controller */
float	Q_STEER_EXT[3]				= { 2.8673f, -2.8469f, .0f };	/* Steer position controller */
float	P_SMITH_SPEED[5]			= {0.2977f,.0f,1,-0.7023f,.0f};	/* {b0,b1,a0,a1,a2} */
int16_t	U_STEER_FEEDFORWARD[2]		= {0,0};						/* Feedforward signal: Weight,other */
int16_t	U_STEER_DECOUPLING[2]		= {0,0};						/* Decoupling signal: battery-charge,speed */

/** Desired setpoint for steering angle.
  * -630:max right, +630: max left
  */
int16_t  SETPOINT_STEER_POS = 0;

/** Desired setpoint for steering angle in Open Loop.
  * -254:max right, +254: max left
  */
int16_t SETPOINT_OPENLOOP_STEER_SPEED = 0;

/** Time of when the setpoint was last changed (1/10 of ms) */
uint32_t SETPOINT_STEER_TIMESTAMP = 0;
uint32_t SETPOINT_OPENLOOP_STEER_TIMESTAMP = 0;

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
		cmd.sampling_period_ms_tenths = SAMPLING_PERIOD_MSth*10;
		init_encoders(cmd);
	}
	// ADC: current sense of steering motor, to ADC0 pin
	{
		TFrameCMD_ADC_start_payload_t cmd;
		cmd.active_channels[0] = CURRENT_SENSE_ADC_CH;
		cmd.use_internal_refvolt = false;
		cmd.measure_period_ms_tenths = SAMPLING_PERIOD_MSth*10;
		adc_process_start_cmd(cmd);
	}
	// ABS ENC:
	// TODO: Refactor sampling period vs. send USB period
	{
		init_EMS22A(ENCODER_ABS_CS,ENCODER_ABS_CLK,ENCODER_ABS_DO,SAMPLING_PERIOD_MSth);
		EMS22A_active = true;
	}
	// Steer signal: PWM:
	gpio_pin_mode(PWM_PIN_NO, OUTPUT);
	pwm_init(PWM_OUT_TIMER, PWM_PRESCALER_1 );  // freq_PWM = F_CPU / (prescaler*510)
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,0x00);
	// Steer direction:
	gpio_pin_mode(PWM_DIR, OUTPUT);
	gpio_pin_write(PWM_DIR, false);
}

void setVerbosityControl(TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control)
{
	global_decimate = verbosity_control;
}

void enableSteerController(bool enabled)
{
	STEERCONTROL_active = enabled;
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
		
	Steer_offset = p.STEER_OFFSET;
	ANTIWINDUP_CTE = p.Antiwindup;
	for (int i=0;i<2;i++)
	{
		U_STEER_DECOUPLING[i]	= p.U_STEER_DECOUPLING[i];
		U_STEER_FEEDFORWARD[i]	= p.U_STEER_FEEDFORWARD[i];
	}
}

void setOpenLoopSetpoint_Steer(int16_t speed)
{
	SETPOINT_OPENLOOP_STEER_SPEED = speed;
	SETPOINT_OPENLOOP_STEER_TIMESTAMP = millis();
}
void setControllerSetpoint_Steer(int16_t pos)
{
	SETPOINT_STEER_POS=pos;
	SETPOINT_STEER_TIMESTAMP = millis();
}

// CONTROL FOR STEERING WHEEL
// Stopwatch: 0.35 ms
void processSteerController()
{
	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis_STEER < CONTROL_sampling_period_ms_tenths)
		return;

	CONTROL_last_millis_STEER = tnow;
	// ========= Encoders calibration algorithm ===================================
	// Incremental encoder reading
	cli();
	const int32_t enc_diff = enc_last_reading.encoders[0];
	sei();
	// Read abs encoder:
	{
		const uint16_t abs_enc_pos_new = enc_abs_last_reading.enc_pos - Steer_offset; // Abs encoder (10 bit resolution)

		// Filter out clearly erroneous readings from the abs encoder: 
		if (abs(abs_enc_pos_new - abs_enc_pos)<1060)
			abs_enc_pos = abs_enc_pos_new;
	}
	// Calibration with absolute encoder:
	const float K_enc_diff = 337.0f / (500.0f * 100.0f);	/** 500: Pulses per revolution
															  * 100: Reductor 100:1
															  * 337: Experimental constant
															  */
	// Incremental encoder calibration if encoders differences are upper than ten pulses
	if ((abs_enc_pos - enc_offset_correction)>10)
		enc_offset_correction = abs_enc_pos - enc_diff * K_enc_diff;
	// Define encoder value to controller
	Encoder_dir[0] = enc_offset_correction + enc_diff * K_enc_diff;

	// Control:
	/*	Speed encoder reading and Smith Predictor implementation*/
	float rpm = (Encoder_dir[0] - Encoder_dir[1]) / T;
	Ys[0] = (- Ys[1] * P_SMITH_SPEED[3] - Ys[2] * P_SMITH_SPEED[4] + P_SMITH_SPEED[0] * U_steer_controller[1+3] + P_SMITH_SPEED[1] * U_steer_controller[2+3])/P_SMITH_SPEED[2];

	if (!STEERCONTROL_active)
	{
		// open-loop mode:
		// Watchdog timer:
		if (tnow>(SETPOINT_OPENLOOP_STEER_TIMESTAMP+ WATCHDOG_TIMEOUT_msth))
			SETPOINT_OPENLOOP_STEER_SPEED = 0;
		U_steer_controller[0] = SETPOINT_OPENLOOP_STEER_SPEED;
	}
	else
	{
		// Automatic mode
	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ */
		if (tnow>(SETPOINT_STEER_TIMESTAMP + WATCHDOG_TIMEOUT_msth))
			SETPOINT_STEER_POS = 0;
	/*	Position reference reading */
		Ref_pos[0]	= SETPOINT_STEER_POS;
	/*	Correction of the wheels' direction */
		Ref_pos[0] = - Ref_pos[0];
	/*	Slope reference limit to over current protection*/
		float pendiente = (Ref_pos[0] - Ref_pos[1]) / T;
		if (pendiente >= sat_ref)
			Ref_pos[0] = (Ref_pos[1] + sat_ref);
	/*	Position error. Extern loop*/
		Error_pos[0] = Ref_pos[0] - Encoder_dir[0];
	/*	Position controller */
		Ref_speed[0] = Ref_speed[1] + Q_STEER_EXT[0] * Error_pos[0] + Q_STEER_EXT[1] * Error_pos[1] + Q_STEER_EXT[2] * Error_pos[2];
	/*	Speed error. Intern loop*/
		Error_speed[0] = Ref_speed[0] - Ys[0] - (rpm - Ys[3]);
	/*	Speed controller */
		U_steer_controller[0] = U_steer_controller[1] + Q_STEER_INT[0] * Error_speed[0] + Q_STEER_INT[1] * Error_speed[1] + Q_STEER_INT[2] * Error_speed[2];
	/*	Control Signal with feedforward & decoupling*/
		U_steer_controller[0] = U_steer_controller[0] + U_STEER_DECOUPLING[0] + U_STEER_DECOUPLING[1] + U_STEER_FEEDFORWARD[0] + U_STEER_FEEDFORWARD[1];
	/*	Variable to Anti-windup technique*/
		int m_v= round(U_steer_controller[0]);
	/*	Saturation */
		bool has_sat = false;
		if (abs(U_steer_controller[0]) > 254)
		{
			if (U_steer_controller[0] < -254)
				U_steer_controller[0] = -254;
			else
				U_steer_controller[0] = 254;
			has_sat = true;
		}
		
		#warning Check max current and stop?
	/*	Anti-windup technique*/ 
		if(has_sat)
			Antiwindup[0] = (U_steer_controller[0] - m_v) / ANTIWINDUP_CTE;
		else
			Antiwindup[0] = 0;
		U_steer_controller[0] = round(0.5 * (2 * U_steer_controller[0] + T * (Antiwindup[0] + Antiwindup[1])));
	}
	/* for both, open & closed loop: protection against steering mechanical limits: */
	if (abs(Encoder_dir[0]) >= steer_mech_limit_pos)
		steer_mech_limit_reached = true;
	else if (abs(Encoder_dir[0]) <= (steer_mech_limit_pos - 50) && steer_mech_limit_reached)
		steer_mech_limit_reached = false;

	// Disallow going further outwards:
	if (steer_mech_limit_reached)
	{
		if(Encoder_dir[0] > 0 && U_steer_controller[0] > 0)
	 		U_steer_controller[0] = 0;
		if(Encoder_dir[0] < 0 && U_steer_controller[0] < 0)
	 		U_steer_controller[0] = 0;
	}

	/* Values actualization*/
	do_shift(Ref_pos);
	do_shift(Antiwindup);
	do_shift(Error_pos);
	do_shift(Ref_speed);
	do_shift(Error_speed);
	do_shift(Encoder_dir);
	do_shift(Ys);
	do_shift(U_steer_controller);

	/*	Direction*/
	bool u_steer_is_positive = (U_steer_controller[0] >= 0);
	uint8_t u_steer = abs(U_steer_controller[0]);

	// Output PWM:
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,u_steer);

	// PWM direction:
	gpio_pin_write(PWM_DIR, u_steer_is_positive);

	TFrame_STEER_CONTROL_SIGNAL tx;
	// Decimate the number of msgs sent to the PC:
	static uint8_t decim0 = 0;
	if (++decim0>global_decimate.decimate_CONTROLSIGNAL)
	{
		decim0=0;
		tx.payload.timestamp_ms_tenth = tnow;
		tx.payload.Steer_control_signal = U_steer_controller[0];
		tx.payload.Encoder_absoluto = abs_enc_pos;
		tx.payload.Encoder_incremental = enc_diff;
		tx.payload.Encoder_signal = Encoder_dir[0];
		tx.payload.Steer_ADC_current_sense = ADC_last_reading.adc_data[0];

		tx.calc_and_update_checksum();

		UART::Write((uint8_t*)&tx,sizeof(tx));
	}
}