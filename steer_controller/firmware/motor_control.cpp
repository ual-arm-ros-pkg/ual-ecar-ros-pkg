/*
 * Motor control file.
 *
 * Created: 20/03/2014 
 *  Author: Jose Luis Blanco Claraco
 */ 

#include "firmware-common.h"


// The desired position as sent from the user from the PC. 
// (in "encoder ticks" units)
volatile int16_t MOTOR_CONTROL_SETPOINT = 0;   

// -----------------------------------------------------
//   Motor control
// -----------------------------------------------------
void realtime_motor_control()
{
	// P controller
	// out = P * ( setpoint - state )
	
#define FIXEDPOINTPRECISION   1000	
	
	// "P" constant  * 1024/100%-duty-cycle
	const int64_t K_P = 1      * FIXEDPOINTPRECISION;  // /10000 is to simulate decimal points
	const int64_t err = -( (int64_t)MOTOR_CONTROL_SETPOINT - (int64_t)ENCODER_READ.ticks_acum);
	int64_t act = (K_P * err)    / FIXEDPOINTPRECISION;
	
	if (act<-1023) act = -1023; 
	if (act>1023) act=1023;
	
	SetMotorPWM((int)act);
}

