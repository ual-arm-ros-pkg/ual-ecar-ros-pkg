/*****************************************************************************************
 FILE: steering-control-firmware2pc-structs.h
 
 Shared file between PC-based C++ code and the embedded firmware.  

Jose Luis Blanco Claraco (C) 2005-2014
Universidad de Almeria
****************************************************************************************/
#ifndef STEER_FIRM_SHARED_DATATYPES_H
#define STEER_FIRM_SHARED_DATATYPES_H

#include <stdlib.h>

#if !defined(__AVR_MEGA__)
#	pragma pack(push, 1) // exact fit - no padding
#endif

#define STEERCONTROL_COMMS_FRAME_START_FLAG  0x69
#define STEERCONTROL_COMMS_FRAME_END_FLAG    0x96

/** List of parameters that can be set from the PC via USB */
struct TFirmwareParams
{
	uint8_t   ENABLE_POS_CONTROL;  //!< true: position control; false: direct PWM control. (Apart from this, there is the CLUTCH set to on/off!)
	uint16_t  READ_ENCODERS_PERIOD; //!< Period of this task, in 0.1ms units
	uint16_t  MOTOR_CONTROL_PERIOD; //!< Period of this task, in 0.1ms units
	uint8_t   SEND_ENCODER_DECIMATION; 
	
	
	// Defaults:
	TFirmwareParams() : 
		ENABLE_POS_CONTROL(0),
		READ_ENCODERS_PERIOD(5),
		MOTOR_CONTROL_PERIOD(5),
		SEND_ENCODER_DECIMATION(100)
	{
	}
};

/** Timely report of controller/encoder state */
struct TStatusReport
{
	uint32_t timestamp;   //!< Current time (in 0.1ms units)
	
	// Encoder position:
	int32_t  encoder_pos; //!< Last absolute encoder position (in ticks)
	
	// Encoder velocity:
	int32_t  encoder_last_incr; //!< Last encoder increment (in ticks)
	uint32_t encoder_last_incr_period; //!< The period for the given increment (in 0.1ms units)
	
	uint16_t current_sense_adc; //!< Last ADC reading from the current sense (Volts = ADC * 2.56/1024)
	
	// Filtered encoder velocity:
	
	TStatusReport()
	{}
};	

enum STEER_COMMS_COMMANDS
{
	CMD_SET_CLUTCH = 0,
	CMD_SET_AUTO_MODE,
	CMD_SET_PWM_VALUE,
	CMD_SET_POS_CONTROL_SETPOINT,
	CMD_SET_REPORT_DECIMATION
};

struct TCmdSetClutch
{
	// HEADER
	uint8_t cmd_code;
	// PAYLOAD
	uint8_t relay_state;  //!< =0:off, !=0:on

	TCmdSetClutch() : cmd_code(CMD_SET_CLUTCH) {}	
};

struct TCmdSetAutoMode
{
	// HEADER
	uint8_t cmd_code;
	// PAYLOAD
	uint8_t enable_pos_control;

	TCmdSetAutoMode() : cmd_code(CMD_SET_AUTO_MODE) {}
};


struct TCmdSetPWMValue
{
	// HEADER
	uint8_t cmd_code;
	// PAYLOAD
	int32_t pwm_value;  //!< Between (-1023, 1023)

	TCmdSetPWMValue() : cmd_code(CMD_SET_PWM_VALUE) {}
};

struct TCmdSetPosControlSetPoint
{
	// HEADER
	uint8_t cmd_code;
	// PAYLOAD
	int16_t setpoint_ticks;  //!< Desired set point, in encoder ticks.

	TCmdSetPosControlSetPoint() : cmd_code(CMD_SET_POS_CONTROL_SETPOINT) {}
};

struct TCmdSetReportDecimation
{
	// HEADER
	uint8_t cmd_code;
	// PAYLOAD
	uint8_t report_decimation;  //!< Number of encoder read cycles to send a status report to the PC via USB

	TCmdSetReportDecimation() : cmd_code(CMD_SET_REPORT_DECIMATION) {}
};



#if !defined(__AVR_MEGA__)
#	pragma pack(pop)
#endif

#endif
