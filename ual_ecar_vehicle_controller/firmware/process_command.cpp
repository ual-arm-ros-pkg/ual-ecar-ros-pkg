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

#include <string.h> // memset()

#include "libclaraquino/leds.h"
#include "libclaraquino/uart.h"
#include "libclaraquino/millis_timer.h"
#include "libclaraquino/delays.h"
#include "libclaraquino/gpio.h"
#include "libclaraquino/adc_internal.h"
#include "libclaraquino/pwm.h"
#include "vehicle_controller_declarations.h"


#include "libclaraquino/mod_dac_max5500.h"
#include "libclaraquino/mod_ems22a.h"

// Fixed pins configuration for this hardware:
#include "libclaraquino/claraquino_config.h"

struct TimeoutData
{
	uint32_t TIMEOUT_TICKS;   //!< Number of millis() ticks to timeout an output signal. Default=1000 ms
	
	bool PWM_any;
	uint32_t PWM_last_changed[2];  //!< Last timestamp (millis()) for each PWM channel

	bool DAC_any;
	uint32_t DAC_last_changed[4];  //!< Last timestamp (millis()) for each DAC channel

	TimeoutData() :
		TIMEOUT_TICKS(1000),
		PWM_any(false),
		DAC_any(false)
	{
		::memset(PWM_last_changed,0, sizeof(PWM_last_changed));
		::memset(DAC_last_changed,0, sizeof(DAC_last_changed));
	}
};

TimeoutData PendingTimeouts;



void send_simple_opcode_frame(const uint8_t op)
{
	const uint8_t rx[] = { FRAME_START_FLAG, op, 0x00, 0x00, FRAME_END_FLAG };
	UART::Write(rx,sizeof(rx));
}

void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data)
{
	switch (opcode)
	{
	case OP_NOP:
	{
		if (datalen!=0) return send_simple_opcode_frame(RESP_WRONG_LEN);

		// No-operation: just a fake command to check if comms are alive
		return send_simple_opcode_frame(RESP_NOP);
	}
	break;
	
	case OP_SET_DAC:
	{
		if (datalen!=sizeof(TFrameCMD_SetDAC_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_SetDAC_payload_t dac_req;
		memcpy(&dac_req,data, sizeof(dac_req));

		// Init upon first usage:
		static bool dac_init = false;
		if (!dac_init)
		{
			mod_dac_max5500_init();
			dac_init = true;
		}
		const uint16_t dac_value = (uint16_t(dac_req.dac_value_HI) << 8) | dac_req.dac_value_LO;
		mod_dac_max5500_update_single_DAC(dac_req.dac_index,dac_value);

		if (dac_req.flag_enable_timeout)
		{
			if (dac_req.dac_index<sizeof(PendingTimeouts.DAC_last_changed)/sizeof(PendingTimeouts.DAC_last_changed[0]))
			{
				PendingTimeouts.DAC_any=true;
				PendingTimeouts.DAC_last_changed[dac_req.dac_index] = millis();
			}
		}

		// send answer back:
		send_simple_opcode_frame(RESP_SET_DAC);
	}
	break;
	case OP_SET_GPIO:
	{
		if (datalen!=sizeof(TFrameCMD_GPIO_output_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		const uint8_t pin_no = data[0];
		const uint8_t pin_val = data[1];
		gpio_pin_mode(pin_no, OUTPUT);
		gpio_pin_write(pin_no, pin_val);

		// send answer back:
		send_simple_opcode_frame(RESP_SET_GPIO);
	}
	break;
	case OP_GET_GPIO:
	{
		if (datalen!=sizeof(TFrameCMD_GPIO_read_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		const uint8_t pin_no = data[0];
		gpio_pin_mode(pin_no, INPUT);
		const uint8_t val = gpio_pin_read(pin_no);

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, RESP_GET_GPIO, 0x01, pin_no, val, uint8_t(0x00 +pin_no+ val)/*checksum*/, FRAME_END_FLAG };
		UART::Write(rx,sizeof(rx));
	}
	break;

	case OP_START_CONT_ADC:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_ADC_start_payload_t adc_req;
		memcpy(&adc_req,data, sizeof(adc_req));

		adc_process_start_cmd(adc_req);
		// send answer back:
		send_simple_opcode_frame(RESP_START_CONT_ADC);
	}
	break;

	case OP_STOP_CONT_ADC:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_stop_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		adc_process_stop_cmd();

		// send answer back:
		send_simple_opcode_frame(RESP_STOP_CONT_ADC);
	}
	break;

	case OP_START_ENCODERS:
	{
		if (datalen!=sizeof(TFrameCMD_ENCODERS_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_ENCODERS_start_payload_t enc_req;
		memcpy(&enc_req,data, sizeof(enc_req));

		init_encoders(enc_req);
		ENCODERS_active=true;

		// send answer back:
		send_simple_opcode_frame(RESP_START_ENCODERS);
	}
	break;

	case OP_STOP_ENCODERS:
	{
		TFrameCMD_ENCODERS_start_payload_t cmd_empty;
		init_encoders(cmd_empty);
		ENCODERS_active=false;

		// send answer back:
		send_simple_opcode_frame(RESP_STOP_ENCODERS);
	}
	break;

	case OP_START_EMS22A:
	{
		if (datalen!=sizeof(TFrameCMD_EMS22A_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_EMS22A_start_payload_t EMS22A_req;
		memcpy(&EMS22A_req,data, sizeof(EMS22A_req));
		if (init_EMS22A(
			EMS22A_req.ENCODER_ABS_CS,EMS22A_req.ENCODER_ABS_CLK, 
			EMS22A_req.ENCODER_ABS_DO, EMS22A_req.sampling_period_ms_tenths
			))
		{

			EMS22A_active = true;
			// send answer back:
			send_simple_opcode_frame(RESP_START_EMS22A);
		}
		else
		{
			// params error:
			return send_simple_opcode_frame(RESP_INVALID_PARAMS);
		}
	}
	break;

	case OP_STOP_EMS22A:
	{
		EMS22A_active = false;
		// send answer back:
		send_simple_opcode_frame(RESP_STOP_EMS22A);
	}
	break;

	case OP_CONTROL_MODE:
	{
		if (datalen!=sizeof(TFrameCMD_CONTROL_MODE_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_CONTROL_MODE_payload_t control_req;
		memcpy(&control_req,data, sizeof(control_req));
		enableSteerController(control_req.steer_enable);
		enableThrottleController(control_req.throttle_enable);
	}
	break;
	case OP_CONTROL_STEERING_SET_PARAMS:
	{
		if (datalen!=sizeof(TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t steer_controller_params;
		memcpy(&steer_controller_params,data, sizeof(steer_controller_params));
		setSteer_ControllerParams(steer_controller_params);
	}
	break;

	case OP_CONTROL_THROTTLE_SET_PARAMS:
	{
		if (datalen!=sizeof(TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t throttle_controller_params;
		memcpy(&throttle_controller_params,data, sizeof(throttle_controller_params));
		setThrottle_ControllerParams(throttle_controller_params);
	}
	break;

	case OP_CONTROL_STEERING_SETPOINT:
	{
		if (datalen!=sizeof(TFrameCMD_CONTROL_STEERING_SETPOINT_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		TFrameCMD_CONTROL_STEERING_SETPOINT_payload_t control_steer_setpoint;
		memcpy(&control_steer_setpoint,data, sizeof(control_steer_setpoint));
		setSteerControllerSetpoint_Steer(control_steer_setpoint.SETPOINT_STEER_POS);
	}
	break;
	
	case OP_OPENLOOP_STEERING_SETPOINT:
	{
		if (datalen!=sizeof(TFrameCMD_OPENLOOP_STEERING_SETPOINT_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		TFrameCMD_OPENLOOP_STEERING_SETPOINT_payload_t ol_steer_setpoint;
		memcpy(&ol_steer_setpoint,data, sizeof(ol_steer_setpoint));
		setSteerOpenLoopSetpoint_Steer(ol_steer_setpoint.SETPOINT_OPENLOOP_STEER_SPEED);
	}
	break;
	
	case OP_OPENLOOP_THROTTLE_SETPOINT:
	{
		if (datalen!=sizeof(TFrameCMD_OPENLOOP_THROTTLE_SETPOINT_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		TFrameCMD_OPENLOOP_THROTTLE_SETPOINT_payload_t ol_throttle_setpoint;
		memcpy(&ol_throttle_setpoint,data, sizeof(ol_throttle_setpoint));
		setSteerOpenLoopSetpoint_VehVel(ol_throttle_setpoint.SETPOINT_OPENLOOP_THROTTLE);
	}
	break;
	
	case OP_CONTROL_THROTTLE_SETPOINT:
	{
		if (datalen!=sizeof(TFrameCMD_CONTROL_THROTTLE_SETPOINT_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		TFrameCMD_CONTROL_THROTTLE_SETPOINT_payload_t control_throttle_setpoint;
		memcpy(&control_throttle_setpoint,data, sizeof(control_throttle_setpoint));
		setSteerControllerSetpoint_VehVel(control_throttle_setpoint.SETPOINT_CONTROL_THROTTLE_SPEED);
	}
	break;

	case OP_VERBOSITY_CONTROL:
	{
		if (datalen!=sizeof(TFrameCMD_VERBOSITY_CONTROL_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control;
		memcpy(&verbosity_control,data, sizeof(verbosity_control));
		setVerbosityControl(verbosity_control);
	}
	break;
	
	default:
	{
		// Error:
		send_simple_opcode_frame(RESP_UNKNOWN_OPCODE);
	}
	break;
	};
}

void process_timeouts()
{
	TimeoutData &pt = PendingTimeouts; // shortcut
	const uint32_t tnow = millis();

	if (pt.DAC_any)
	{
		pt.DAC_any=false; // if no timeout is set, don't waste time in the next time we are called.
		for (uint8_t i=0;i<sizeof(pt.DAC_last_changed)/sizeof(pt.DAC_last_changed[0]);i++)
		{
			if (pt.DAC_last_changed[i]!=0)
			{
				pt.DAC_any=true;
				if (tnow - pt.DAC_last_changed[i] > pt.TIMEOUT_TICKS)
				{
					// Watchdog timer event!
					pt.DAC_last_changed[i]=0; // reset this one
					mod_dac_max5500_update_single_DAC(i,0);
				}
				
			}
		}
	}
	
	if (pt.PWM_any)
	{
		pt.PWM_any=false; // if no timeout is set, don't waste time in the next time we are called.
		for (uint8_t i=0;i<sizeof(pt.PWM_last_changed)/sizeof(pt.PWM_last_changed[0]);i++)
		{
			if (pt.PWM_last_changed[i]!=0)
			{
				pt.PWM_any=true;
				if (tnow - pt.PWM_last_changed[i] > pt.TIMEOUT_TICKS)
				{
					// Watchdog timer event!
					pt.PWM_last_changed[i]=0; // reset this one
					pwm_set_duty_cycle(PWM_TIMER0,pwm_pin_t(i),0x00);
				}
			}
		}
	}
}
