/*****************************************************************************************
 FILE: commands.h
Jose Luis Blanco Claraco (C) 2005-2014
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/
#ifndef STEERCONTROL_FIRM_COMMON_H
#define STEERCONTROL_FIRM_COMMON_H
// --------- Firmware specific definitions  ---------------
#define STEERCONTROL_FIRMWARE_VERSION "2.1.0"
// --------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

#include "CUART.h"

// XTAL: 16.0 Mhz clock
#define F_CPU 16000000UL
#include <util/delay.h>

#include "LEDs.h"

extern void InitPWM();
/** Set the motor "speed" (pwm-controlled). 
  *  "pwm" is in the range: 
  *  Min=-1023 -> Full speed in one direction
  *  Max=+1023 -> Full speed in the other direction
  *  Zero = 0  -> STOP
  */
extern void SetMotorPWM(int pwm, bool update_last_cmd = true);

struct TEncoderRead
{
	volatile uint32_t timestamp;
	volatile int16_t  ticks_incr;  // signed!
	volatile uint32_t time_incr;   // timestamp increment since last pkg
	volatile int32_t  ticks_acum;  // signed!
	
	TEncoderRead():
	timestamp(0),
	ticks_incr(0),
	time_incr(0),
	ticks_acum(0) { }
};
extern TEncoderRead ENCODER_READ;
extern volatile uint16_t ADC_CURRENT_SENSE_READ; // The last value of the current sense ADC reading

#include "public_include/steering-control-firmware2pc-structs.h"
extern TFirmwareParams firm_params;  //!< Set of current params, updated from the PC via USB commands.
extern int16_t  last_pwm_cmd;  //!< Latest PWM cmd, in either manual PWM or P-controlled mode.

#endif
