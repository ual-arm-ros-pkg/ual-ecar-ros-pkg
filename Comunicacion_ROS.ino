/*Programa que realiza la lectura del puerto serie */
#include <ros.h>
#include <std_msgs/Float64.h>
#include <LiquidCrystal.h>

int pinLectura = 48;

ros::NodeHandle nh;

//Estas funciones del ejemplo "pubsub.ino" de la librería ros.h no me interesa emplearlas, ¿harían falta?

void messageCb(const std_msgs::Float64& steer_controller_pwm_led_msg){
  digitalWrite(pinLectura, HIGH-digitalRead(pinLectura));   // blink the led
}

//Esta forma es como se define los topics a los que se subscribe en el ejemplo "pubsub.ino" de la librería ros.h
ros::Subscriber<std_msgs::Float64> pwm_value("steer_controller_pwm_led", messageCb);

// Este método está basado en el código de los nodos de ROS de github
//m_sub_pwm_value = m_nh.subscribe("steer_controller_pwm", float  );

//ros::Subscriber m_sub_pwm_value;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7); 

void setup() {
  pinMode(pinLectura, OUTPUT);
  //Inicialización realizada en el ejemplo "pubsub.ino" de la librería ros.h
  nh.initNode();
  nh.subscribe(pwm_value);
  lcd.begin(16, 2);
  Serial.begin(1000000);
  pinMode(pinLectura, OUTPUT);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
  lcd.write("Wait 2s");
  delay(2000);
  lcd.clear();
  lcd.write("Reading ...");
}
 
void loop() {
// char hello[13] = "hello world!";
  double x;
/*str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
*/
  lcd.clear();
  lcd.write("E-CARM. ROS ");
  lcd.setCursor(0, 1);
  lcd.write("Pwm_value: ");
  x=pwm_value;
  lcd.write(x);
  delay(100);
}
