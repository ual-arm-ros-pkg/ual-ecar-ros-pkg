/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <Wire.h>
#include <SPI.h>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Pin mapping:
const int PIN_DAC_MAX5500_CS  = 9;
const int PIN_LED             =  13;


/* SPI frames for the MAX5500 chip
           16 bits
     MSB --- LSB (MSB first)
|A1 A0  |  C1 C0 | D11 ... D0 |

Commands C1.C0:
 * 0 1: Load DAC, do NOT update DAC output.
 * 1 1: Load DAC, update ALL DAC outputs.
 
- POLARITY: CPOL=0 (inactive SCK=low level)
- PHASE: CPHA=1 (master changes data on the falling edge of clock)
 ==> SPI_MODE_0 (1?)
 
*/

void mod_dac_max5500_send_spi_word(uint16_t tx_word)
{
	// Send HiByte:
	SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
	
	// nCS -> 0
	digitalWrite(PIN_DAC_MAX5500_CS, LOW);
	delayMicroseconds(1); // Min. 40ns: nCS->0 to SCK

	SPI.transfer16 (tx_word);
	
	delayMicroseconds(1);
	
	// nCS -> 1
	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);

	SPI.endTransaction();
}

void mod_dac_max5500_update_single_DAC(uint8_t dac_idx, uint16_t dac_value)
{
	// See word format at the top of this file
	const uint16_t tx_word =
	(((uint16_t)dac_idx) << 14) |
	(((uint16_t)0x03)    << 12) |
	(dac_value & 0x0fff);
	

	Serial.print("set DAC:");
	Serial.print(dac_idx);
	Serial.print(" ");
	Serial.println(dac_value);

	mod_dac_max5500_send_spi_word(tx_word);
}


void setup() 
{
	pinMode(PIN_LED, OUTPUT);

	// start the SPI library:
	SPI.begin();

	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);  // /CS=1
	pinMode(PIN_DAC_MAX5500_CS, OUTPUT);

	// Set all chip outputs to 0V
	mod_dac_max5500_send_spi_word(0x8000);

	Serial.begin(9600);
}

void loop()
{
	for (uint16_t val=0; ; val+=100)
	{
		mod_dac_max5500_update_single_DAC(0,val);
		//Serial.println("Iteration...\n");
		digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(5);              // wait for a second
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(1);
	}
}
