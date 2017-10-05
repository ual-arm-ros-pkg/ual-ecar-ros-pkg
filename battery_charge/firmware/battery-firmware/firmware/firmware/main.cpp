/*
 * firmware.cpp
 *
 * Created: 04/10/2017 10:46:02
 * Author : Francisco José Mañas
 */ 

#include <Arduino.h>
#include <avr/io.h>
#include <Wire.h>
#include <SPI.h>
#include <config.h>
#include <batterycharge_declarations.h>


void setup()
{
	Serial.begin(115200);
}

void loop()
{
	processIncommingPkts();

	processBattery();

	// Handle possible timeouts of previous commands:
	process_timeouts();
}

