/*Programa que realiza la lectura del puerto serie */
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <LiquidCrystal.h>

ros::NodeHandle nh;

//Estas funciones del ejemplo "pubsub.ino" de la librería ros.h no me interesa emplearlas, ¿harían falta?

void messageCb(const std_msgs::Float64& steer_controller_pwm_msg){
  digitalWrite(2, HIGH-digitalRead(2));   // blink the led
}
void messageCa(const std_msgs::Bool& arduino_daq_GPIO_output7_msg){
  digitalWrite(2, HIGH-digitalRead(2));   // blink the led
}

//Esta forma es como se define los topics a los que se subscribe en el ejemplo "pubsub.ino" de la librería ros.h
ros::Subscriber<std_msgs::Float64> pwm_value("steer_controller_pwm",messageCb);
ros::Subscriber<std_msgs::Bool> rev_buttom("arduino_daq_GPIO_output7", messageCa);
// Este método está basado en el código de los nodos de ROS de github
//m_sub_pwm_value = m_nh.subscribe("steer_controller_pwm", Float64  );

//ros::Subscriber m_sub_pwm_value;


std_msgs::String str_msg;
ros::Publisher func("func", &str_msg);

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

char control[16] = "ROS: Conexion Ok";

void setup() {
  pinMode(2, OUTPUT);
  //Inicialización realizada en el ejemplo "pubsub.ino" de la librería ros.h
  nh.initNode();
  nh.advertise(func);
  nh.subscribe(pwm_value);
  nh.subscribe(rev_buttom);
  lcd.begin(16, 2);
  Serial.begin(1000000);
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
/*str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
*/str_msg.data = control;
  func.publish( &str_msg );
  lcd.clear();
  lcd.write("E-CARM. ROS ");
  lcd.setCursor(0, 1);
  lcd.write(control);
  delay(100);
}
