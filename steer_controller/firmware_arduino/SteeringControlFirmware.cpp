/*
 * Main file for the steering control board of ARM's eCar
 *
 * Created: 20/03/2014 
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


uint32_t getTimer_us()  // 100ths of us
{
	cli();
	const uint32_t ret = timer_us;
	sei();
	return ret;
}

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

volatile uint16_t  READ_ENCODERS_PERIOD_CNT  = 0;
// const    uint16_t  READ_ENCODERS_PERIOD      = 20;  --> "params."
volatile bool      READ_ENCODERS_NEW = false;  // means that a new read is ready in ENCODER_READ

volatile uint16_t  MOTOR_CONTROL_PERIOD_CNT = 0;
//const    uint16_t  MOTOR_CONTROL_PERIOD     = 20;  --> "params."

TEncoderRead ENCODER_READ;
volatile uint16_t ADC_CURRENT_SENSE_READ=1;

void realtime_update_encoder_read();
void realtime_motor_control();

bool RUN_realtime_update_encoder_read = false;
bool RUN_realtime_motor_control = false;


// Handle the Timer 2 events:
SIGNAL(TIMER0_COMPA_vect)
{
	// Update real-time high freq clock:
	++timer_us;
	
#if  0
	// hard real time tasks:
	if (++READ_ENCODERS_PERIOD_CNT>=firm_params.READ_ENCODERS_PERIOD)
	{
		RUN_realtime_update_encoder_read = true;
		READ_ENCODERS_PERIOD_CNT = 0;
		
	}
#endif 

	if (++MOTOR_CONTROL_PERIOD_CNT>=firm_params.MOTOR_CONTROL_PERIOD)
	{
		RUN_realtime_motor_control = true;
		MOTOR_CONTROL_PERIOD_CNT=0;
	}
	
	// If ADC is done, restart it:
	if ((ADCSRA & (1<<ADSC)) == 0)
	{
		ADC_CURRENT_SENSE_READ = ADC;
		sbi(ADCSRA, ADSC);
	}
	
	
}

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

uint16_t blocking_readADC(uint8_t channel)
{
	// Channel selection:
	ADMUX = (ADMUX & ~(0x0F)) | channel;
	
	// Start:
	ADCSRA |= (1<<ADSC);
		
	while (ADCSRA & (1<<ADSC))
	{		
	}
	
	return ADC;	
}

#if 0

void encoders_cntsclr_set()  // 0=SET
{
	cbi(PORTD,7);
	sbi(DDRD,7);
}
void encoders_cntsclr_unset()  // 0=SET
{
	sbi(PORTD,7);
	sbi(DDRD,7);
}

void encoders_output_enable()
{
	cbi(PORTA,2);
	sbi(DDRA,2);
}	
void encoders_output_disable()
{
	sbi(PORTA,2);
	sbi(DDRA,2);
}


void encoders_init()
{
	encoders_cntsclr_set();
	__builtin_avr_delay_cycles(5);
	encoders_cntsclr_unset();
	encoders_output_disable();	
	
	// Enable ext. interrupt in PINA7=PCINT7 --> ENCODER Z+ 
	sbi( PCMSK0, PCINT7 );
	sbi( PCICR, PCIE0 );
}


volatile bool encoder_Z_pulse_detected = false;

// Handle Encoder Z signal (zero)
SIGNAL(PCINT0_vect)
{
	// PINA7 = Encoder Z 
	encoder_Z_pulse_detected = true;
	encoders_cntsclr_set();
	__builtin_avr_delay_cycles(2);
	encoders_cntsclr_unset();
}

void realtime_update_encoder_read()
{
	// Save time increment:
	ENCODER_READ.time_incr = timer_us-ENCODER_READ.timestamp;
	// And new timestamp:
	ENCODER_READ.timestamp = timer_us;
	
	DDRB = 0x00;
	DDRC = 0x00;

	encoders_output_enable();
	__builtin_avr_delay_cycles(2);
	ENCODER_READ.ticks_incr = PINB | (PINC<<8);
	//encoders_output_disable();
	
	if (PINA & (1 << PINA1))
		ENCODER_READ.ticks_incr = -ENCODER_READ.ticks_incr;

	if (encoder_Z_pulse_detected) 
	{
		encoder_Z_pulse_detected = false;
		ENCODER_READ.ticks_acum = 0;		
	}		 
	
	ENCODER_READ.ticks_acum += ENCODER_READ.ticks_incr;
		
	encoders_cntsclr_set();
	__builtin_avr_delay_cycles(2);
	encoders_cntsclr_unset();
}

uint8_t SEND_ENCODER_DECIMATION_cnt = 0;
#endif


namespace UART 
{
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

	for (int i=0;i<3;i++)
	{
		LED_ON(6);  _delay_ms(30);
		LED_OFF(6); _delay_ms(30);
	}	
	// Enable interrupts:
	sei();
	
	while(1)
	{
		// Real-time cyclic tasks:
		// --------------------------------
#if 0
		if (RUN_realtime_update_encoder_read)
		{
			realtime_update_encoder_read();
			RUN_realtime_update_encoder_read = false;
			READ_ENCODERS_NEW=true;
		}
#endif
		if (RUN_realtime_motor_control && firm_params.ENABLE_POS_CONTROL!=0)
		{
			realtime_motor_control();
			RUN_realtime_motor_control = false;
		}


#if 0
		if (READ_ENCODERS_NEW)
		{
			READ_ENCODERS_NEW=false;
			
			if (++SEND_ENCODER_DECIMATION_cnt==firm_params.SEND_ENCODER_DECIMATION)
			{
				SEND_ENCODER_DECIMATION_cnt=0;
				
				TStatusReport rep;
				
				rep.timestamp   = ENCODER_READ.timestamp;
				rep.encoder_pos = ENCODER_READ.ticks_acum;
				rep.encoder_last_incr = ENCODER_READ.ticks_incr;
				rep.encoder_last_incr_period = ENCODER_READ.time_incr;
				rep.current_sense_adc = ADC_CURRENT_SENSE_READ;
								
				UART::WriteBinaryFrame((const uint8_t*)&rep, sizeof(TStatusReport));
				
				//sprintf(s,"[%lu] ENC=%ld | INCR=%d | TIM.INCR=%lu\r\n",
					//(long unsigned int)ENCODER_READ.timestamp,
					//(long signed int) ENCODER_READ.ticks_acum,
					//(signed int) ENCODER_READ.ticks_incr,
					//(long unsigned int)ENCODER_READ.time_incr				
					//);  // 300us
				//UART::WriteStringFramed(s);
			}							
		}
#endif

		if (UART_LAST_RX_CMD_NEW)
		{
			process_command(UART_LAST_RX_CMD, UART_LAST_RX_CMD_LEN);
			UART_LAST_RX_CMD_NEW=false;
		}
		// Overcurrent protection:
		if (!OVERCURRENT_TRIGGERED)
		{
			if (  ADC_CURRENT_SENSE_READ > OCUR_CENTRAL_PT+OVERCURRENT_THRESHOLD_ADC ||
					ADC_CURRENT_SENSE_READ < OCUR_CENTRAL_PT-OVERCURRENT_THRESHOLD_ADC)
			{
				// Increase overcurrent timer counter:
				const uint32_t tim_now = getTimer_us();
				if (tim_now!=OVERCURRENT_LAST_TRIGGERED_TIM) {
					++OVERCURRENT_ELLAPSED_100US;
					OVERCURRENT_LAST_TRIGGERED_TIM = tim_now;
				}
			
				// If timer > limit: 
				if (OVERCURRENT_ELLAPSED_100US >= OVERCURRENT_TIME_THRESHOLD_MS * 10)
				{
					OVERCURRENT_TRIGGERED = true;
					// Store the sign of the PWM motion direction, so we can detect a change and reset the triggered flag:
					OVERCURRENT_PWM_POSITIVE_WHEN_TRIGGERED = (last_pwm_cmd>0);
					SetMotorPWM(0, false /* dont update sign of last pwm cmd */);
				}
			}
			else
			{
				OVERCURRENT_ELLAPSED_100US = 0;
			}
		}
		else
		{
			// We are in overcurrent protection:
			SetMotorPWM(0, false /* dont update sign of last pwm cmd */);
		}
	}
}


