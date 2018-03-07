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

#include "vehicle_controller_throttle_declarations.h"
#include "libclaraquino/mod_dac_max5500.h"
#include "libclaraquino/millis_timer.h"
#include "libclaraquino/uart.h"
#include "libclaraquino/pwm.h"
#include "libclaraquino/gpio.h"
#include "math.h"
#include <avr/interrupt.h> // sei()

// ============== HARDWARE CONFIGURATION =====================
const uint16_t SAMPLING_PERIOD_MSth = 10 /*ms*/ *10 /*th*/;
const int8_t ENCODER_DIFF_A         = 0x42; // PD2
const int8_t ENCODER_DIFF_B         = 0x43; // PD3
const int8_t CURRENT_SENSE_ADC_CH   = 0;    // ADC #0
const int8_t THROTTLE_FEEDBACK_ADC_CH= 2;   // ADC #2
const int8_t PWM_DIR                = 0x44; // CW/CCW
const int8_t RELAY_FRWD_REV         = 0x11;
const int8_t RELAY_PEDAL_INTERLOCK  = 0x13;
#define PWM_OUT_TIMER               PWM_TIMER2   // PD6=OC2B
#define PWM_OUT_PIN                 PWM_PIN_OCnB
const int8_t PWM_PIN_NO             = 0x46;
const int8_t DAC_OUT_PIN_NO         = 0x24; // PB4
// ===========================================================

const uint32_t WATCHDOG_TIMEOUT_msth = 1000*10;  // timeout for watchdog timer (both, openloop & controller, vehvel & steer)

// Control vars:
float	T				=	0.01f;			// Sample time
float	U_throttle_controller[6] = {0,0,0,0,0,0};
TFrameCMD_VERBOSITY_CONTROL_payload_t global_decimate;

// Auxiliary vars:
uint32_t  CONTROL_last_millis_THROTTLE = 0;
uint16_t  CONTROL_sampling_period_ms_tenths = 50 /*ms*/ * 10;
bool	  THROTTLECONTROL_active = false; //true: controller; false: open loop

float	Q_THROTTLE[3]				= {0,0,0};
int16_t	U_THROTTLE_FEEDFORWARD[2]	= {0,0}; /*Weight,other*/
int16_t U_THROTTLE_DECOUPLING		= 0; /*battery-charge*/

/** Desired setpoint for throttle in Open Loop.
  * [-1,0]V:max reverse, [0,+1]V: max forward
  */
float SETPOINT_OPENLOOP_THROTTLE = .0f;

/** Desired setpoint for throttle in Open Loop.
  * 0:min speed, 12.5 m/s: max forward
  */
float SETPOINT_CONTROL_THROTTLE_SPEED = .0f;

/** Time of when the setpoint was last changed (1/10 of ms) */
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
		cmd.sampling_period_ms_tenths = SAMPLING_PERIOD_MSth*10;
		init_encoders(cmd);
	}

	// ADC: current sense of steering motor, to ADC0 pin
	{
		TFrameCMD_ADC_start_payload_t cmd;
		cmd.active_channels[0] = CURRENT_SENSE_ADC_CH;
		cmd.active_channels[1] = THROTTLE_FEEDBACK_ADC_CH;
		cmd.use_internal_refvolt = false;
		cmd.measure_period_ms_tenths = SAMPLING_PERIOD_MSth*10;
		adc_process_start_cmd(cmd);
	}

	// Init DAC:
	mod_dac_max5500_init(DAC_OUT_PIN_NO);

	// PWM:
	gpio_pin_mode(PWM_PIN_NO, OUTPUT);
	pwm_init(PWM_OUT_TIMER, PWM_PRESCALER_1 );  // freq_PWM = F_CPU / (prescaler*510)
	pwm_set_duty_cycle(PWM_OUT_TIMER,PWM_OUT_PIN,0x00);
	// PWM direction:
	gpio_pin_mode(PWM_DIR, OUTPUT);
	gpio_pin_write(PWM_DIR, false);

	// Relay:
	gpio_pin_mode(RELAY_PEDAL_INTERLOCK, OUTPUT);
	gpio_pin_write(RELAY_PEDAL_INTERLOCK, false);

}

void setVerbosityControl(TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control)
{
	global_decimate = verbosity_control;

}

void enableThrottleController(bool enabled)
{
	THROTTLECONTROL_active = enabled;
}

void setThrottle_ControllerParams(const TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t &p)
{
	for (int i=0;i<3;i++)
		Q_THROTTLE[i] = p.Q_THROTTLE_CONTROLLER[i];

	for (int i=0;i<2;i++)
		U_THROTTLE_FEEDFORWARD[i] = p.U_THROTTLE_FEEDFORWARD[i];

	U_THROTTLE_DECOUPLING = p.U_THROTTLE_DECOUPLING;
}

void setOpenLoopSetpoint_VehVel(float ol_vel_mps)
{
	SETPOINT_OPENLOOP_THROTTLE = ol_vel_mps;
	SETPOINT_OPENLOOP_THROTTLE_TIMESTAMP = millis();
}
void setControllerSetpoint_VehVel(float vel_mps)
{
	SETPOINT_CONTROL_THROTTLE_SPEED = vel_mps;
	SETPOINT_CONTROL_THROTTLE_SPEED_TIMESTAMP = millis();
}

// CONTROL FOR MAIN VEHICLE MOTOR
// Stopwatch: 0.35 ms
void processThrottleController()
{
	const uint32_t tnow = millis();
	if (tnow-CONTROL_last_millis_THROTTLE < CONTROL_sampling_period_ms_tenths)
		return;

	CONTROL_last_millis_THROTTLE = tnow;
	cli();
	const int32_t enc_diff = enc_last_reading.encoders[0];
	sei();
	
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
	float pedal = .0f; /* [-1,1] */
	if (!THROTTLECONTROL_active)
	{
		if (tnow>(SETPOINT_OPENLOOP_THROTTLE_TIMESTAMP + WATCHDOG_TIMEOUT_msth))
			SETPOINT_OPENLOOP_THROTTLE = 0;

		pedal = SETPOINT_OPENLOOP_THROTTLE;
	}
	else
	{
		if (tnow>(SETPOINT_CONTROL_THROTTLE_SPEED_TIMESTAMP + WATCHDOG_TIMEOUT_msth))
			SETPOINT_CONTROL_THROTTLE_SPEED = 0;

		// TODO: Throttle-by-wire controller here!!
		pedal = SETPOINT_CONTROL_THROTTLE_SPEED / 12.5;
	}

	// Ensure normalized speed is in range [-1,1]
	if (pedal>1) pedal=1;
	else if (pedal<-1) pedal=-1;

	// Output direction:
	// Relay output = HIGH if going BACKWARDS.
	U_throttle_controller[0] = pedal * 5;
	gpio_pin_write(RELAY_FRWD_REV,(pedal<0));

	// Pedal enable relay (to Curtis controller J1-8 pin)
	/*gpio_pin_write(RELAY_PEDAL_INTERLOCK,(abs(pedal)>0.1f));*/
	gpio_pin_write(RELAY_PEDAL_INTERLOCK,(abs(pedal)<0.1f));
	#warning Cambio solo para comprobar que funciona el rele

	// Output value:
	uint16_t veh_speed_dac = abs(U_throttle_controller[0]* 4095/5.0); // 12bit DAC constant
	mod_dac_max5500_update_single_DAC(0 /*DAC idx*/, veh_speed_dac);

	/* Values actualization*/
	do_shift(U_throttle_controller);
	
	TFrame_CONTROL_SIGNAL tx;
	// Decimate the number of msgs sent to the PC:
	static uint8_t decim0 = 0;
	if (++decim0>global_decimate.decimate_CONTROLSIGNAL)
	{
		decim0=0;
		tx.payload.timestamp_ms_tenth = tnow;
		tx.payload.Throttle_control_signal = U_throttle_controller[0];
		tx.payload.Throttle_analog_feedback = ADC_last_reading.adc_data[1];
		tx.payload.Encoder_incremental = enc_diff;
		tx.payload.Steer_ADC_current_sense = ADC_last_reading.adc_data[0];

		tx.calc_and_update_checksum();

		UART::Write((uint8_t*)&tx,sizeof(tx));
	}
}
