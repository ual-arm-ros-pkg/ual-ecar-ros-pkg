/*
 * Main file for the steering control board of ARM's eCar
 *
 * Created: 1/06/2017 
 *  Author: Jose Luis Blanco Claraco
 */ 

#include "firmware-common.h"
#include "commands.h"

// Define baud rate
#define USART_BAUDRATE   115200 // 1000000 //115200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1)

volatile uint32_t timer_us=0;

uint16_t OVERCURRENT_THRESHOLD_ADC = (uint16_t)( 1.5f * 1024/5.0f );
bool     OVERCURRENT_TRIGGERED = false;    // If overcurrent protection has been triggered. If true, PWM is forced to 0
bool     OVERCURRENT_PWM_POSITIVE_WHEN_TRIGGERED = false;  // true if PWM>0,  false if PWM<0
const uint16_t OCUR_CENTRAL_PT = uint16_t(2.5f*1024/5.0f);

uint32_t  OVERCURRENT_LAST_TRIGGERED_TIM = 0;
uint16_t  OVERCURRENT_ELLAPSED_100US     = 0;
uint16_t  OVERCURRENT_TIME_THRESHOLD_MS  = 300;



// Use timer 0: to a period of 100us
void configureMillisecsTimer()
{
	timer_us=0;
	// Timer2: 8bits, 
	// prescaler = 8
	// 1 overflow with compare = (1+199): 8* 200 / 16MHz = 100 us
	
	// Setup Timer
	TCNT0 = 0;

	TCCR0A =  
	(0 << COM0A1) | 
	(0 << COM0A0) |
	(0 << COM0B1) |
	(0 << COM0B0) |
	(1 << WGM01) |
	(0 << WGM00); // mode 2: CTC

	TCCR0B =  
	(0 << FOC0A) | // No input capture, so no noise canceler
	(0 << FOC0B) |
	(0 << WGM02) |
	(0 << CS02) |
	(1 << CS01) |  // Prescaler 000 = off, 10 = no prescale, 11 = 8, 10+11 = 64, 12 = 256, 12+10 = 1024 12+11+x = external clock.(x determins rising/falling edge)
	(0 << CS00);

	OCR0A = 199;
	
	// Enable interrupt: 
	TIMSK0 |= (1<<OCIE0A);
}


volatile uint16_t  MOTOR_CONTROL_PERIOD_CNT = 0;
//const    uint16_t  MOTOR_CONTROL_PERIOD     = 20;  --> "params."

volatile uint16_t ADC_CURRENT_SENSE_READ=1;

void realtime_motor_control();

bool RUN_realtime_motor_control = false;


// Handle ADC read event:
//SIGNAL(ADC_vect)

void configureADCs()
{
	//Avcc(+5v) as voltage reference: REFS0=1, REFS1=0
	//TODO: Volt. ref= internal 2.56V
	// TODO: Mirar y documentar aqui que canal ADC es (parece que el ADC0)
	ADMUX = (1<<REFS0) | (0<<REFS1);
	
	// Autotrigger:
	ADCSRB = 0; // Free-running mode

	//Prescaler at 128 -> 16Mhz/128 = 125Khz ADC clock
	ADCSRA = 
	   ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)) 
	   | (1<<ADSC) // Start the first conversion
	   | (1<<ADEN) // Power up
	   //| (1<<ADIE) // ADC Interrupt enable
	   ;	
}

namespace UART {
	void QUEUE_RX_BYTE(const uint8_t rx);
}


int main(void)
{
	UART::Configure(true,true,false,0,BAUD_PRESCALE);
	UART::InitInterruptBasedCommandReceiver();

	UART::WriteStringFramed( "Steering Control Firmware-Build " __TIMESTAMP__ "\r\n" );
	//encoders_init();
	configureADCs();
	InitPWM();

	configureMillisecsTimer();

	for (int i=0;i<3;i++) {
		LED_ON(6);  _delay_ms(30);
		LED_OFF(6); _delay_ms(30);
	}	
	// Enable interrupts:
	sei();
	
	while(1) {
		realtime_motor_control();

		if (UART_LAST_RX_CMD_NEW) {
			process_command(UART_LAST_RX_CMD, UART_LAST_RX_CMD_LEN);
			UART_LAST_RX_CMD_NEW=false;
		}
		
	}
}


