#include <Wire.h>
#include <SPI.h>

const int ledPin =  13;      // the number of the LED pin

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);   

  Serial.println("Iteration...\n");
}
