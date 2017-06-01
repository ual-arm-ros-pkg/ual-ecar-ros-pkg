﻿/*
 * main.cpp
 *
 * Created: 24/12/2013 10:40:56
 *  Author: Jose Luis
 */ 


/*
 Examples:
 
 T1_FREQ_ON 250
 T1_FREQ_OFF

*/

#include "firmware-common.h"

TFirmwareParams firm_params;
int16_t  last_pwm_cmd = 0;

// Trick: faster than strlen()
//template <typename CHAR,int N> int static_strlen(const CHAR (&)[N] array){ return N-1; }

template<typename T, int size>
int static_strlen(T(&)[size]){return size-1;}


#define STARTS_WITH(_STR,_COMPARE_TO) ( 0==strncmp(_STR,_COMPARE_TO,static_strlen(_COMPARE_TO))  )


extern volatile int16_t MOTOR_CONTROL_SETPOINT;   // in "encoder ticks" units


// void set_relay_on();
// void set_relay_off();

/** Process a command from the PC and send any required response back.
  * \return false on error, unknown command, etc.
  */
bool process_command(const uint8_t *cmd, const uint16_t cmd_len) {
	switch (cmd[0]) {
		case CMD_SET_CLUTCH: {
// 			const TCmdSetClutch *frame = (TCmdSetClutch*)cmd;
// 			if (frame->relay_state) {
// 				set_relay_on();
// 				LED_ON(6);
// 			}					
// 			else {
// 				set_relay_off();
// 				LED_OFF(6);
// 			}				
		}
		break;
		
		case CMD_SET_AUTO_MODE: {
			const TCmdSetAutoMode *frame = (TCmdSetAutoMode*)cmd;
			firm_params.ENABLE_POS_CONTROL = frame->enable_pos_control ? 1:0;
		}
		break;

		case CMD_SET_OVERCURRENT_THRESHOLD: {
// 			const TCmdSetOvercurrentThreshold *frame = (TCmdSetOvercurrentThreshold*)cmd;
// 			OVERCURRENT_THRESHOLD_ADC = (uint16_t)( frame->threshold_volt * 1024 / 5.0f);
// 			OVERCURRENT_TIME_THRESHOLD_MS = frame->threshold_ms_length;
		}
		break;

		case CMD_SET_PWM_VALUE: {
			const TCmdSetPWMValue *frame = (TCmdSetPWMValue*)cmd;
// 			if (OVERCURRENT_TRIGGERED) {
// 				// shall we reset the o.c. flag?
// 				if (frame->pwm_value==0 || (OVERCURRENT_PWM_POSITIVE_WHEN_TRIGGERED && frame->pwm_value<0) || (!OVERCURRENT_PWM_POSITIVE_WHEN_TRIGGERED && frame->pwm_value>0) ) {
// 					OVERCURRENT_TRIGGERED = false;
// 				}
// 			}
// 			
// 			if (!OVERCURRENT_TRIGGERED) {
				SetMotorPWM(frame->pwm_value);
/*			}*/
		}
		break;

		case CMD_SET_POS_CONTROL_SETPOINT: {
			const TCmdSetPosControlSetPoint *frame = (TCmdSetPosControlSetPoint*)cmd;
			OVERCURRENT_TRIGGERED = false;

			MOTOR_CONTROL_SETPOINT = frame->setpoint_ticks;
		}
		break;

		case CMD_SET_REPORT_DECIMATION: {
			const TCmdSetReportDecimation *frame = (TCmdSetReportDecimation*)cmd;
			firm_params.SEND_ENCODER_DECIMATION = frame->report_decimation;
		}
		break;
	
		default:
			//UART::WriteStringFramed( "ERR Unknown command\r\n" );
			//return false; // No known command!
		break;	
	}		
	
	return true;	
}


// pololu  < --- >  Atmel328P <----------------------------------->	Atmega2560
//   DIR              A5(PC5)										A5(PF5) Corresponde con ADC5 pero no con PCINT5(Seria el PB5, pero ese coincide con el 0C1A)
//   PWMH             PWM~D9 (PB1)									PWM~D11 (PB5:OC1A)
//   PWML             PWM~D10 (PB2)  (=1 for forward/backward)		PWM~D10 (PB6:OC1B)

/* PWM output on PB6/OC1B for left motor, PB5/OC1A for right motor; these
   pins are connected to H-bridge; we just need to send signals */
#define SetupLPWM()	{ SetBit(DDRB, DDB6);  SetBit(DDRB, DDB6);  }
#define SetupRPWM()	{ SetBit(DDRB, DDB5);  SetBit(DDRC, DDC5); } /*No se muy bien porque en el SetupLPWM los dos SetBit son iguales y en este emplea el C5!!!!!!!!!!!*/
/* we compare to OCR1A/B for R/L motor speeds */
//#define lPWM		OCR1B ¿Porque está comentado el lPWM??????
#define rPWM		OCR1A
 /*No tengo claro como se configurarían estas lineas */
#define RFwd()		( ClearBit(PORTC, PORTC5),   SetBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0) )
#define RRev()		(   SetBit(PORTC, PORTC5),   SetBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0) )
#define RStop()		( ClearBit(PORTC, PORTC5), ClearBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0) )


/* sets up microprocessor for PWM control of motors */
void InitPWM() {
// Setup Timer
  TCNT1 = 0;

  /* see comment above for info on PWM initialization */
  /* start with motors disconnected from Timer/Counter output */

/*
So, if I run this at 18.432Mhz no prescaler, 
10Bit mode, I should be able to get to 18kHz
8Bit Mode, 8x prescale = 9kHz
OutFreq = (InFreq/prescale)/Bits 

See page 134 of Mega324p datasheet

*/


  TCCR1A =  (0 << COM1A1) | // Set for normal mode for now, macros above change it for directions.
            (0 << COM1A0) |
            (0 << COM1B1) |
            (0 << COM1B0) |
            (1 << WGM11) |
            (1 << WGM10); // Fast PWM, (10+12 = 8bit, 10+11+12 = 10bit)

  TCCR1B =  (0 << ICNC1) | // No input capture, so no noise canceler
            (0 << ICES1) |
            (0 << WGM13) |
            (1 << WGM12) |
            (0 << CS12) |
            (0 << CS11) |  // Prescaler 000 = off, 10 = no prescale, 11 = 8, 10+11 = 64, 12 = 256, 12+10 = 1024 12+11+x = external clock.(x determins rising/falling edge)
            (1 << CS10);

  TCCR1C = 0x00;  // Set the Force bits to 0 for PWM modes.

  //TIMSK1 =  (0 << ICIE1) | // Input Capture Interrupt Enable
            //(0 << OCIE1B) | // Output Compare B Match Interrupt Enable
            //(0 << OCIE1A) | // Output Compare A Match Interrupt Enable
            //(0 << TOIE1); // Timer 1 Overflow interrupt enable
//
  /* set up ports */
  //SetupLPWM();
  SetupRPWM();
  
  // 
  sbi(DDRB, DDB6); 
  sbi(PORTB, PORTB6);

  /* OCR1A/B are the values that the timer is compared to; a match will
     cause the output to change; small values mean the motor runs for a
     short period (slower); larger values are longer times (faster)*/
  //lPWM = 
  rPWM = 0;	// (value is irrelevant since outputs are disconnected)
}

/* pwm values can range from -255 (full-speed reverse)
   to 255 (full-speed forward), with 0 indicating a stop */
void SetMotorPWM(int pwm, bool update_last_cmd) {
	if (update_last_cmd) {
		last_pwm_cmd = pwm;
	}
	if (pwm == 0){
		RStop();
	}
	else {
		if (pwm >= 0) {
			RFwd();
		}
		else {
			RRev();
			pwm = -pwm;
		}
		if (pwm > 1023)
			pwm = 1023;
    rPWM = pwm;		// set width for PWM
  }
}

/*¿Que objetivo tienen estas funciones?*/
// void set_relay_on() {
// 	sbi(DDRD,DDD3);
// 	cbi(PORTD,PORTD3);	
// }	
// 
// void set_relay_off() {
// 	sbi(DDRD,DDD3);
// 	sbi(PORTD,PORTD3);
// }
