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

#include <stdlib.h>

#include <stdint.h>

#if !defined(__AVR_MEGA__)
#	pragma pack(push, 1) // exact fit - no padding
#endif

/*
START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96
*/

#define FRAME_START_FLAG  0x69
#define FRAME_END_FLAG    0x96

enum opcode_t {
	// -----------------------------
	// COMMANDS PC -> uC
	// -----------------------------
	OP_NOP             = 0x00,
	OP_SET_DAC         = 0x10,
	OP_SET_GPIO        = 0x11,
	OP_GET_GPIO        = 0x12,
	OP_START_CONT_ADC  = 0x20,
	OP_STOP_CONT_ADC   = 0x21,
	OP_START_ENCODERS  = 0x30,
	OP_STOP_ENCODERS   = 0x31,
	OP_START_EMS22A	   = 0x40, //
	OP_STOP_EMS22A     = 0x41, //
	// Control:
	OP_CONTROL_MODE					= 0x50,
	OP_CONTROL_STEERING_SET_PARAMS	= 0x51,
	OP_CONTROL_THROTTLE_SET_PARAMS	= 0x52,
	OP_CONTROL_STEERING_SETPOINT	= 0x53,
	OP_OPENLOOP_STEERING_SETPOINT	= 0x54,
	OP_OPENLOOP_THROTTLE_SETPOINT	= 0x55,
	OP_CONTROL_THROTTLE_SETPOINT	= 0x56,
	OP_VERBOSITY_CONTROL			= 0X57,

	// -----------------------------
	// Responses uC -> PC
	// -----------------------------
	RESP_OFFSET = 0x70,
	// -----------------------------
	RESP_NOP              = OP_NOP + RESP_OFFSET,
	RESP_SET_DAC          = OP_SET_DAC + RESP_OFFSET,
	RESP_SET_GPIO         = OP_SET_GPIO + RESP_OFFSET,
	RESP_GET_GPIO         = OP_GET_GPIO + RESP_OFFSET,
	RESP_START_CONT_ADC   = OP_START_CONT_ADC + RESP_OFFSET,
	RESP_STOP_CONT_ADC    = OP_STOP_CONT_ADC + RESP_OFFSET,
	RESP_START_ENCODERS   = OP_START_ENCODERS + RESP_OFFSET,
	RESP_STOP_ENCODERS    = OP_STOP_ENCODERS + RESP_OFFSET,
	RESP_START_EMS22A     = OP_START_EMS22A + RESP_OFFSET, //
	RESP_STOP_EMS22A      = OP_STOP_EMS22A + RESP_OFFSET,  //
	RESP_ADC_READINGS     = 0x92,
	RESP_ENCODER_READINGS = 0x93,
	RESP_EMS22A_READINGS  = 0x94,
	RESP_CONTROL_SIGNAL   = 0x95,
	RESP_CPU_USAGE_STATS  = 0xA0,

	// error codes:
	RESP_CHECKSUM_ERROR    = 0xfa,
	RESP_FRAME_ERROR       = 0xfb,
	RESP_INVALID_PARAMS    = 0xfc,
	RESP_WRONG_LEN         = 0xfd,
	RESP_UNKNOWN_OPCODE    = 0xfe
};


template <typename Payload>
struct TBaseFrame
{
    const uint8_t  START_FLAG;
    const uint8_t  OPCODE;
    const uint8_t  DATALEN;
    // --------- Payload -------
    Payload  payload;
    // -------------------------
    uint8_t  CHECKSUM;
    const uint8_t  END_FLAG;

    // Defaults:
    TBaseFrame(uint8_t opcode) :
            START_FLAG(FRAME_START_FLAG),
            OPCODE(opcode),
            DATALEN(sizeof(Payload)),
            END_FLAG(FRAME_END_FLAG)
    {
    }

    void calc_and_update_checksum()
    {
        CHECKSUM = calc_checksum();
    }

    uint8_t calc_checksum() const
    {
        const uint8_t len   = DATALEN; //reinterpret_cast<const uint8_t*>(ptr_frame)[2];
        const uint8_t *data = reinterpret_cast<const uint8_t*>(&payload);
        uint8_t ret =0;
        for (unsigned int i=0;i<len;i++) ret+=*data++;
        return ret;
    }
};

struct TFrameCMD_NOP_payload_t
{
};
struct TFrameCMD_NOP : public TBaseFrame<TFrameCMD_NOP_payload_t>
{
	TFrameCMD_NOP() : TBaseFrame(OP_NOP)
	{
	}
};

struct TFrameCMD_SetDAC_payload_t
{
    uint8_t  dac_index;
    uint8_t  dac_value_HI, dac_value_LO;
    uint8_t  flag_enable_timeout : 1;   // bitfield: 1 bit flag
    TFrameCMD_SetDAC_payload_t() :
        dac_index(0), dac_value_HI(0), dac_value_LO(0),flag_enable_timeout(0)
    {
    }
};
struct TFrameCMD_SetDAC : public TBaseFrame<TFrameCMD_SetDAC_payload_t>
{
    // Defaults:
    TFrameCMD_SetDAC() : TBaseFrame(OP_SET_DAC)
    {
    }
};

struct TFrameCMD_GPIO_output_payload_t
{
    uint8_t  pin_index;
    uint8_t  pin_value;
};
struct TFrameCMD_GPIO_output : public TBaseFrame<TFrameCMD_GPIO_output_payload_t>
{
    // Defaults:
    TFrameCMD_GPIO_output() : TBaseFrame(OP_SET_GPIO)
    {
    }
};

struct TFrameCMD_GPIO_read_payload_t
{
	uint8_t  pin_index;
};
struct TFrameCMD_GPIO_read : public TBaseFrame<TFrameCMD_GPIO_read_payload_t>
{
	// Defaults:
	TFrameCMD_GPIO_read() : TBaseFrame(OP_GET_GPIO)
	{
	}
};


struct TFrameCMD_ADC_start_payload_t
{
	/** Fill all the pins (0=ADC0, 1=ADC1, ...) that want to get read with an ADC. Default values = -1, means ignore that channel.
	  */
	int8_t   active_channels[8];
	uint16_t measure_period_ms_tenths; //!<  in tenths of milliseconds Default = 2000
	uint8_t  use_internal_refvolt; //!< 0 or 1. Default=0

	TFrameCMD_ADC_start_payload_t() :
		measure_period_ms_tenths(2000),
		use_internal_refvolt(0)
	{
		for (int i=0;i<8;i++) {
			active_channels[i]=-1;
		}
	}
};
struct TFrameCMD_ADC_start : public TBaseFrame<TFrameCMD_ADC_start_payload_t>
{
	// Defaults:
	TFrameCMD_ADC_start() : TBaseFrame(OP_START_CONT_ADC)
	{
	}
};

struct TFrameCMD_ADC_stop_payload_t
{
};
struct TFrameCMD_ADC_stop : public TBaseFrame<TFrameCMD_ADC_stop_payload_t>
{
	// Defaults:
	TFrameCMD_ADC_stop() : TBaseFrame(OP_STOP_CONT_ADC)
	{
	}
};

struct TFrame_ADC_readings_payload_t
{
	uint32_t timestamp_ms_tenths; //!< timestamp, in tenths of milliseconds
	uint16_t adc_data[8];
};
struct TFrame_ADC_readings : public TBaseFrame<TFrame_ADC_readings_payload_t>
{
	// Defaults:
	TFrame_ADC_readings() : TBaseFrame(RESP_ADC_READINGS)
	{
	}
};


struct TFrameCMD_ENCODERS_start_payload_t
{
	static const uint8_t NUM_ENCODERS = 2;

	/** Fill pin numbers ("0xAB numbering") that want to get used as quadrature encoder A,B & Z channels. 
	  * Leave to "0" if don't need Z channels or one of the A/B encoder channels.
	  */
	int8_t encA_pin[NUM_ENCODERS], encB_pin[NUM_ENCODERS], encZ_pin[NUM_ENCODERS];
	uint16_t sampling_period_ms_tenths;

	TFrameCMD_ENCODERS_start_payload_t() :
		sampling_period_ms_tenths(2500)
	{
		for (uint8_t i=0;i<NUM_ENCODERS;i++) {
			encA_pin[i]=encB_pin[i]=encZ_pin[i]=0;
		}
	}
};
struct TFrameCMD_ENCODERS_start : public TBaseFrame<TFrameCMD_ENCODERS_start_payload_t>
{
	// Defaults:
	TFrameCMD_ENCODERS_start() : TBaseFrame(OP_START_ENCODERS)
	{
	}
};

struct TFrameCMD_ENCODERS_stop_payload_t
{
};
struct TFrameCMD_ENCODERS_stop : public TBaseFrame<TFrameCMD_ENCODERS_stop_payload_t>
{
	// Defaults:
	TFrameCMD_ENCODERS_stop() : TBaseFrame(OP_STOP_ENCODERS)
	{
	}
};


struct TFrame_ENCODERS_readings_payload_t
{
	uint32_t timestamp_ms_tenths;
	int32_t  encoders[2];
	uint32_t period_ms_tenths;
};
struct TFrame_ENCODERS_readings : public TBaseFrame<TFrame_ENCODERS_readings_payload_t>
{
	// Defaults:
	TFrame_ENCODERS_readings() : TBaseFrame(RESP_ENCODER_READINGS)
	{
	}
};

struct TFrame_ENCODER_ABS_reading_payload_t
{
	uint32_t timestamp_ms_tenths;
	uint16_t enc_pos; //!< Absolute value read from the encoder (10 bits resolution)
	uint8_t  enc_status; //!< See EMS22A datasheet for the bit map
};
struct TFrame_ENCODER_ABS_reading : public TBaseFrame<TFrame_ENCODER_ABS_reading_payload_t>
{
	// Defaults:
	TFrame_ENCODER_ABS_reading() : TBaseFrame(RESP_EMS22A_READINGS)
	{
	}
};

struct TFrame_CONTROL_SIGNAL_payload_t
{
	uint32_t timestamp_ms_tenth;
	int16_t Steer_control_signal;
	uint16_t Throttle_control_signal;
	int32_t Encoder_incremental;
	uint32_t Encoder_absoluto;
	int32_t Encoder_signal;
	uint16_t ADC_signal;
};
struct TFrame_CONTROL_SIGNAL : public TBaseFrame<TFrame_CONTROL_SIGNAL_payload_t>
{
	// Default:
	TFrame_CONTROL_SIGNAL() : TBaseFrame(RESP_CONTROL_SIGNAL)
	{
	}
};

struct TFrame_CPU_USAGE_STATS_payload_t
{
	uint32_t timestamp_ms_tenths;
	/** min/max/average execution time of the uC main loop (in tenths of milliseconds) */
	uint32_t loop_min_time, loop_max_time, loop_average_time;
	uint16_t loop_iterations;
	void clear()
	{
		loop_min_time = 0xffffffff;
		loop_max_time = loop_average_time = 0;
		loop_iterations = 0;
	}
};
struct TFrame_CPU_USAGE_STATS : public TBaseFrame<TFrame_CPU_USAGE_STATS_payload_t>
{
	// Defaults:
	TFrame_CPU_USAGE_STATS() : TBaseFrame(RESP_CPU_USAGE_STATS)
	{
	}
};

struct TFrameCMD_EMS22A_start_payload_t
{
	int8_t ENCODER_ABS_CS, ENCODER_ABS_CLK, ENCODER_ABS_DO;
	uint16_t sampling_period_ms_tenths;

	TFrameCMD_EMS22A_start_payload_t() :
		sampling_period_ms_tenths(500)
	{
		ENCODER_ABS_CS=ENCODER_ABS_CLK=ENCODER_ABS_DO=0;
	}
};

struct TFrameCMD_EMS22A_start : public TBaseFrame<TFrameCMD_EMS22A_start_payload_t>
{
	// Defaults:
	TFrameCMD_EMS22A_start() : TBaseFrame(OP_START_EMS22A)
	{
	}
};

struct TFrameCMD_EMS22A_stop_payload_t
{
};
struct TFrameCMD_EMS22A_stop : public TBaseFrame<TFrameCMD_EMS22A_stop_payload_t>
{
	// Defaults:
	TFrameCMD_EMS22A_stop() : TBaseFrame(OP_STOP_EMS22A)
	{
	}
};

struct TFrameCMD_CONTROL_MODE_payload_t
{
	bool steer_enable; //!< false: open loop (no control algorithm)
	bool throttle_enable; //!< false: open loop (no control algorithm)
};
struct TFrameCMD_CONTROL_MODE : public TBaseFrame<TFrameCMD_CONTROL_MODE_payload_t>
{
	// Defaults:
	TFrameCMD_CONTROL_MODE() : TBaseFrame(OP_CONTROL_MODE)
	{
	}
};


struct TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t
{
	float Q_STEER_INT[3] = {0,0,0}, Q_STEER_EXT[3]={0,0,0}, P_SMITH_SPEED[5] = {0,0,0,0,0};
	int16_t U_STEER_FEEDFORWARD[2]={0,0}/*Weight,other*/, U_STEER_DECOUPLING[2]={0,0}; /*battery-charge,speed*/
};
struct TFrameCMD_CONTROL_STEERING_SET_PARAMS : public TBaseFrame<TFrameCMD_CONTROL_STEERING_SET_PARAMS_payload_t>
{
	// Defaults:
	TFrameCMD_CONTROL_STEERING_SET_PARAMS() : TBaseFrame(OP_CONTROL_STEERING_SET_PARAMS)
	{
	}
};

struct TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t
{
	float Q_THROTTLE_CONTROLLER[3] = {0,0,0};
	int16_t U_THROTTLE_FEEDFORWARD[2]={0,0} /*Weight,other*/, U_THROTTLE_DECOUPLING = 0; /*battery-charge*/
};
struct TFrameCMD_CONTROL_THROTTLE_SET_PARAMS : public TBaseFrame<TFrameCMD_CONTROL_THROTTLE_SET_PARAMS_payload_t>
{
	// Defaults:
	TFrameCMD_CONTROL_THROTTLE_SET_PARAMS() : TBaseFrame(OP_CONTROL_THROTTLE_SET_PARAMS)
	{
	}
};

struct TFrameCMD_CONTROL_STEERING_SETPOINT_payload_t
{
	/** Desired setpoint for steering angle. 
	  * -512:max right, +511: max left
	  */
	int16_t  SETPOINT_STEER_POS { 0 };
};
struct TFrameCMD_CONTROL_STEERING_SETPOINT : public TBaseFrame<TFrameCMD_CONTROL_STEERING_SETPOINT_payload_t>
{
	// Defaults:
	TFrameCMD_CONTROL_STEERING_SETPOINT() : TBaseFrame(OP_CONTROL_STEERING_SETPOINT)
	{
	}
};

struct TFrameCMD_OPENLOOP_STEERING_SETPOINT_payload_t
{
	/** Desired setpoint for steering angle in Open Loop. 
	  * -254:max right, +254: max left
	  */
	int16_t  SETPOINT_OPENLOOP_STEER_SPEED { 0 };
};
struct TFrameCMD_OPENLOOP_STEERING_SETPOINT : public TBaseFrame<TFrameCMD_OPENLOOP_STEERING_SETPOINT_payload_t>
{
	// Defaults:
	TFrameCMD_OPENLOOP_STEERING_SETPOINT() : TBaseFrame(OP_OPENLOOP_STEERING_SETPOINT)
	{
	}
};

struct TFrameCMD_OPENLOOP_THROTTLE_SETPOINT_payload_t
{
/** Desired setpoint for throttle in Open Loop. 
  * [-1,0]:max reverse, [0,1]: max forward
  */
	float  SETPOINT_OPENLOOP_THROTTLE { .0f };
};
struct TFrameCMD_OPENLOOP_THROTTLE_SETPOINT : public TBaseFrame<TFrameCMD_OPENLOOP_THROTTLE_SETPOINT_payload_t>
{
	// Defaults:
	TFrameCMD_OPENLOOP_THROTTLE_SETPOINT() : TBaseFrame(OP_OPENLOOP_THROTTLE_SETPOINT)
	{
	}
};

struct TFrameCMD_CONTROL_THROTTLE_SETPOINT_payload_t
{
	/** Desired setpoint for throttle in Open Loop. 
	  * 0:min speed, 12.5 m/s: max forward
	  */
	float  SETPOINT_CONTROL_THROTTLE_SPEED { .0f };
};
struct TFrameCMD_CONTROL_THROTTLE_SETPOINT : public TBaseFrame<TFrameCMD_CONTROL_THROTTLE_SETPOINT_payload_t>
{
	// Defaults:
	TFrameCMD_CONTROL_THROTTLE_SETPOINT() : TBaseFrame(OP_CONTROL_THROTTLE_SETPOINT)
	{
	}
};

struct TFrameCMD_VERBOSITY_CONTROL_payload_t
{
	/** 
	  */
	uint8_t decimate_ADC { 10 };
	uint8_t decimate_ENCABS {10};
	uint16_t decimate_CPU {10000};
	uint8_t decimate_CONTROLSIGNAL {10};
};
struct TFrameCMD_VERBOSITY_CONTROL : public TBaseFrame<TFrameCMD_VERBOSITY_CONTROL_payload_t>
{
	// Defaults:
	TFrameCMD_VERBOSITY_CONTROL() : TBaseFrame(OP_VERBOSITY_CONTROL)
	{
	}
};

#if !defined(__AVR_MEGA__)
#	pragma pack(pop)
#endif
