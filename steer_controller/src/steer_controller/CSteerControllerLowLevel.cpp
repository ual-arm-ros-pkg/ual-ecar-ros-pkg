/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <steer_controller/CSteerControllerLowLevel.h>
#include <mrpt/system/threads.h> // for sleep()
#include <ros/console.h>
#include <steer_controller/SteerControllerStatus.h>
#include <functional>
#include <cstring>
#include <array>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

float Eje_x = 0;
float Eje_y = 0;
bool Status_mode = false;
bool GPIO7 = false;
int dep = 0;

CSteerControllerLowLevel::CSteerControllerLowLevel() :
	mrpt::utils::COutputLogger("CSteerControllerLowLevel"),
	m_nh_params("~")
{
}

CSteerControllerLowLevel::~CSteerControllerLowLevel()
{
}

bool CSteerControllerLowLevel::initialize()
{
	ROS_INFO("CSteerControllerLowLevel::inicialize() ok.");
	
	m_pub_rev_relay		= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_rev_steering	= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output4", 10);
	m_pub_pwm_steering	= m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm3", 10);
	m_pub_voltage_pedal	= m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);

	m_sub_contr_status	= m_nh.subscribe("steer_controller_status", 10, &CSteerControllerLowLevel::statusCallback, this);
	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &CSteerControllerLowLevel::ejexCallback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &CSteerControllerLowLevel::ejeyCallback, this);
	m_sub_rev_relay		= m_nh.subscribe("arduino_daq_GPIO_output7",10, &CSteerControllerLowLevel::GPIO7Callback, this);

	// Inicialization
	{
		std_msgs::Bool b;
		b.data = true;
		m_pub_rev_relay.publish(b);
		m_pub_rev_steering.publish(b);
	}

	{
		std_msgs::UInt8 msg_ui;
		msg_ui.data = 0;
		m_pub_pwm_steering.publish(msg_ui);
	}

	{
		std_msgs::Float64 msg_f;
		msg_f.data = 1.0;
		m_pub_voltage_pedal.publish(msg_f);
	}
}

bool CSteerControllerLowLevel::iterate()
{
	// Variables
	vector<float> error;
	vector<float> controller_params;
	vector<float> SP_params, SP_out;
	int pwm_steering;
	float voltaje_pedal,aux;
	bool b2,b1,b3;
	// Lectura del modo de control
	bool ok = Status_mode;

	std_msgs::UInt8 msg_ui;
	std_msgs::Float64 msg_f;
	std_msgs::Bool msg_b;

	// If para la division de los modos de control
	// --------------------------------------------

	// Modo manual
	if (ok)
	{	// Este if es para que solo se muestre el mensaje la primera vez que entra en el controlador
		if (dep == 0)
		{
			ROS_INFO("Controlador eCAR en modo manual");
			dep = 1;
		}

		// PWM
		aux = (float)(Eje_x * 254);
		if (aux < 0)
		{
			aux = - aux;
			msg_b.data = false;
		}
		else
		{
			aux = aux;
			msg_b.data = true;
		}
		pwm_steering = (int)(aux);
		m_pub_rev_steering.publish(msg_b);
		msg_ui.data = pwm_steering;
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", pwm_steering);

		// DAC
		voltaje_pedal = 1.0 + Eje_y * 4.76;

		if (voltaje_pedal < 1)
		{
			voltaje_pedal = 1;
		}

		msg_f.data = voltaje_pedal;
		m_pub_voltage_pedal.publish(msg_f);

		ROS_INFO("Pedal: %.02f volts", voltaje_pedal);

		// Bool
		
		b1 = GPIO7;
		msg_b.data = GPIO7;
		m_pub_rev_relay.publish(msg_b);

		ROS_INFO("Rev Relay = %s", b1 ? "true":"false");
	}

	// Modo automatico
	else
	{	// Este if es para que solo se muestre el mensaje la primera vez que entra en el controlador
		if (dep == 1)
		{
			ROS_INFO("Controlador eCAR en modo automatico");
			dep = 0;
		}
		
	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ 
	*/
	/*	Lectura de la referencia de posicion */
		//R_steer = (float)(Eje_x * MAX_ang);
	
	/*	Determinar el Predictor de Smith de la posición*/
		// yp[1] = 1.7788 * yp[2] - 0.7788 * yp[3] + 0.0058 * up[5] + 0.0053 * up[6];
	/*	Calculo del error al restar la restar el encoder de la interior iteracion a la referencia de posicion */
		//error[1] = R_steer - yp[1];

	/*	Introduccion de la ecuacion del controlador (Dos controladores) */
			//	Controlador Lazo interno
			// up[1] = up[2] + q0 * error[1] + q1 * error[2] + q2 * error[3];

	/*	Implementar saturacion */

	/*	Determinar el Predictor de Smith de la velocidad*/
		// ys[1] = ys[2] * __

	/*	Calcular el error del segundo lazo restando el valor de la velocidad determinada en la iteracion anterior */
		//error[4] = up[1] - (ys[1] + realimentación del encoder)

	/*	Introduccion de la ecuacion del controlador */
		// us[1] = us[2] + q0 * error[4] + q1 * error[5];

	/*	Implementar saturacion */

	/*	Actualizar los valores de todos los vactores para la siguiente iteración*/


	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
		voltaje_pedal = 1.0 + Eje_y * 4.76;

		if (voltaje_pedal < 1)
		{
			voltaje_pedal = 1;
		}

		msg_f.data = voltaje_pedal;
		m_pub_voltage_pedal.publish(msg_f);

		ROS_INFO("Pedal: %.02f volts", voltaje_pedal);

	}
		return true;
}


void CSteerControllerLowLevel::statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("Status Mode: %s", msg->data ? "true":"false" );
	Status_mode = msg->data;
}

void CSteerControllerLowLevel::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	//ROS_INFO("Steer_axis %.02f", msg->data );
	Eje_x = msg->data;
}

void CSteerControllerLowLevel::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	//ROS_INFO("Voltage pedal %.02f", msg->data );
	Eje_y = msg->data;
}

void CSteerControllerLowLevel::GPIO7Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("Reverse throttle direcction : %s", msg->data ? "true":"false" );
	GPIO7 = msg->data;
}