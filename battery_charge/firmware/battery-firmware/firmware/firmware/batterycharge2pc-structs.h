/*
 * batterycharge2pc-structs.h
 *
 * Created: 04/10/2017 12:11:13
 *  Author: Francisco José Mañas
 */ 

#pragma once

#include <stdlib.h>

#if !defined(__AVR_MEGA__)
#	pragma pack(push, 1) // exact fit - no padding
#endif

/*
START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96

## Computer => controller
* 0x10: Set DAC value. DATA_LEN = 3
	* DATA[0]   = DAC index
	* DATA[1:2] = DAC value (0x0000-0xffff)  (MSByte first!)
* 0x11: Set GPIO pin. DATA_LEN = 2
	* DATA[0]   = Arduino-based pin number
	* DATA[1]   = 0/1
* 0x12: Read GPIO pin. DATA_LEN = 1
	* DATA[0]   = Arduino-based pin number
* 0x20: Start ADC continuous acquisition task
* 0x21: Stop ADC task
*/

#define FRAME_START_FLAG  0x69
#define FRAME_END_FLAG    0x96

enum opcode_t {
	// -----------------------------
	// COMMANDS PC -> Arduino
	// -----------------------------
	OP_NOP             = 0x00,
	OP_SET_OPTO        = 0x10,
	OP_GET_OPTO        = 0x11,
	OP_START_BAT       = 0x20,
	OP_STOP_BAT        = 0x21,

	// -----------------------------
	// Responses Arduino -> PC
	// -----------------------------
	RESP_OFFSET = 0x70,
	// -----------------------------
	RESP_NOP              = OP_NOP + RESP_OFFSET,
	RESP_SET_OPTO         = OP_SET_OPTO + RESP_OFFSET,
	RESP_GET_OPTO         = OP_GET_OPTO + RESP_OFFSET,
	RESP_START_BAT        = OP_START_BAT + RESP_OFFSET,
	RESP_STOP_BAT         = OP_STOP_BAT + RESP_OFFSET,
	RESP_BAT_READINGS     = 0x92,

	// error codes:
	RESP_CHECKSUM_ERROR    = 0xfa,
	RESP_FRAME_ERROR       = 0xfb,
	RESP_INVALID_PARAMS    = 0xfc,
	RESP_WRONG_LEN         = 0xfd,
	RESP_UNKNOWN_OPCODE    = 0xfe
};
