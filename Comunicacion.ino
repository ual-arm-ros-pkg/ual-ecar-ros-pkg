/*Programa que realiza la lectura del puerto serie */
#include <LiquidCrystal.h>
int pinLectura = 48;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7); 
// Variables dinámicas
 
void setup() {
  lcd.begin(16, 2);
  Serial.begin(1000000);
  pinMode(pinLectura, OUTPUT);
  lcd.print("Starting...");
  delay(2000);
  lcd.clear();
  lcd.write("Wait 4s");
  delay(1000);
  lcd.clear();
  lcd.write("Wait 3s");
  delay(1000);
  lcd.clear();
  lcd.write("Wait 2s");
  delay(1000);
  lcd.clear();
  lcd.write("Wait 1s");
  delay(1000);
  lcd.clear();
  lcd.write("Reading ...");
}
 
void loop() {
  char dato;
 
    /*Se comprueba que exista algún dato en el puerto*/
    if(Serial.available ()){
      lcd.clear();
      lcd.write("Serial Port:");
      lcd.setCursor(0, 1);
      while(Serial.available()>0){
        digitalWrite(pinLectura, HIGH);
        lcd.write(Serial.read());
      }
      digitalWrite(pinLectura, LOW);
    }
}
