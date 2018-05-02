/*
 * batterycharge2pc-structs.h
 *
 * Created: 04/10/2017 12:11:13
 *  Author: Francisco Jose Manas
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

## Computer => controller
* 0x10: Set GPIO pin. DATA_LEN = 2
	* DATA[0]   = Arduino-based pin number
	* DATA[1]   = 0/1
* 0x20: Start ADC continuous acquisition task
* 0x21: Stop ADC task
*/

#define FRAME_START_FLAG  0x69
#define FRAME_END_FLAG    0x96

enum opcode_t {
	// -----------------------------
	// COMMANDS PC -> Arduino
	// -----------------------------
	OP_NOP					= 0x00,
	OP_SET_OPTO				= 0x10,
	OP_START_CONT_ADC		= 0x20,
	OP_STOP_CONT_ADC		= 0x21,
	OP_VERBOSITY_CONTROL	= 0X60,
	// -----------------------------
	// Responses uC -> PC
	// -----------------------------
	RESP_OFFSET = 0x70,
	// -----------------------------
	RESP_NOP              = OP_NOP + RESP_OFFSET,
	RESP_SET_OPTO         = OP_SET_OPTO + RESP_OFFSET,
	RESP_START_CONT_ADC   = OP_START_CONT_ADC + RESP_OFFSET,
	RESP_STOP_CONT_ADC    = OP_STOP_CONT_ADC + RESP_OFFSET,
	RESP_BAT_READINGS     = 0x92,
	RESP_ADC_READINGS     = 0x93,
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

struct TFrameCMD_OPTO_output_payload_t
{
	uint8_t  opto_mask; //!< from 0x01 to 0x10 enable each opto.

};
struct TFrameCMD_OPTO_output : public TBaseFrame<TFrameCMD_OPTO_output_payload_t>
{
    // Defaults:
    TFrameCMD_OPTO_output() : TBaseFrame(OP_SET_OPTO)
    {
    }
};

struct TFrameCMD_ADC_start_payload_t
{
	/** Fill all the pins (0=ADC0, 1=ADC1, ...) that want to get read with an ADC. Default values = -1, means ignore that channel.
	  */
	int8_t   active_channels[3];
	uint16_t measure_period_ms_tenths; //!<  in tenths of milliseconds Default = 2000
	uint8_t  use_internal_refvolt; //!< 0 or 1. Default=0

	TFrameCMD_ADC_start_payload_t() :
		measure_period_ms_tenths(2000),
		use_internal_refvolt(0)
	{
		for (int i=0;i<3;i++) {
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
	uint16_t adc_data[3];
};
struct TFrame_ADC_readings : public TBaseFrame<TFrame_ADC_readings_payload_t>
{
	// Defaults:
	TFrame_ADC_readings() : TBaseFrame(RESP_ADC_READINGS)
	{
	}
};

struct TFrame_BATTERY_readings_payload_t
{
	uint32_t timestamp_ms_tenths;
	int16_t  bat_volts[8];
	uint32_t period_ms_tenths;
};
struct TFrame_BATTERY_readings : public TBaseFrame<TFrame_BATTERY_readings_payload_t>
{
	// Defaults:
	TFrame_BATTERY_readings() : TBaseFrame(RESP_BAT_READINGS)
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

struct TFrameCMD_VERBOSITY_CONTROL_payload_t
{
	/*
	*/
	uint8_t decimate_BAT{10};
	uint16_t decimate_CPU{10000};
	uint8_t decimate_ADC{10};
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


