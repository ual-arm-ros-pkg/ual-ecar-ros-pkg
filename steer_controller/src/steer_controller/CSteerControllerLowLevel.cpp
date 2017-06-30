/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <steer_controller/CSteerControllerLowLevel.h>
#include <mrpt/system/threads.h> // for sleep()
#include <ros/console.h>
// #include <steer_controller/SteerControllerStatus.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

CSteerControllerLowLevel::~CSteerControllerLowLevel()
{
}

bool CSteerControllerLowLevel::initialize()
{
	ROS_INFO("CSteerControllerLowLevel::inicialize() ok.");
	
	m<std_msgs::Bool>("steer_controller_status", 10);
	m_pub_rev_relay		= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_rev_steering	= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output4", 10);
	m_pub_pwm_steering	= m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm3", 10);
	m_pub_voltage_pedal	= m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);

	m_sub_contr_status	= m_nh.subscribe("steer_controller_status", 10, &CSteerControllerLowLevel::status_Callback, this);
	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &CSteerControllerLowLevel::eje_x_Callback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &CSteerControllerLowLevel::eje_y_Callback, this);
	m_sub_rev_relay		= m_nh.subscribe("arduino_daq_GPIO_output7",10, &CSteerControllerLowLevel::GPIO7_Callback, this);

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
	int pwm_steering, b2;
	float voltaje_pedal;
	bool b1;

	// Lectura del modo de control
	bool ok = m_sub_contr_status;

	// If para la división de los modos de control
	// --------------------------------------------

	// Modo manual
	if (!ok)
	{
		ROS_INFO("Controlador eCAR en modo manual");

		std_msgs::UInt8 msg_ui;
		std_msgs::Float64 msg_f;
		std_msgs::Bool msg_b1;
		std_msgs::Bool msg_b2;

		// PWM
		pwm_steering = (int)(m_sub_eje_x * 254);

		msg_ui.data = pwm_steering;
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", pwm_steering);

		// DAC
		voltaje_pedal = 1.0 + m_sub_eje_y * 5.76;

		if (voltaje_pedal<0)
		{
			voltaje_pedal = 0;
		}

		msg_f.data = voltaje_pedal;
		m_pub_voltage_pedal.publish(msg_f);

		ROS_INFO("Pedal: %.02f volts", voltaje_pedal);

		// Bool

		msg_b1.data = m_sub_rev_relay;
		m_pub_rev_relay.publish(msg_b1);

		b1 = (int) m_sub_rev_relay;
		if (pwm_steering < 0)
		{
			b2 = 1;
			msg_b2.data = true;
		}
		else
		{
			b2 = 0;
			msg_b2.data = false;
		}
		m_pub_rev_steering.publish(msg_b2);


		ROS_INFO("B1=%i B2=%i", b1, b2);
		return true;
	}

	// Modo automático
	else
	{
		ROS_INFO("Controlador eCAR en modo automático");
	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ 
	*/
	/*	Lectura de la referencia de posición */
		R_steer = m_sub_steer_ref;
	/*	Cálculo del error al restar la restar el encoder de la interior iteración a la referencia de posición */
		error[1] = R_steer - steer;
	/*	Introducción de la ecuación del controlador (Dos controladores) */
			//	Controlador Lazo 1:
				

	/*	Implementar saturación y transferencia sin salto */

	/*	Registrar la señal de control de este lazo */

	/*	Pasar la señal por un filtro (Trabajar con dos lineas) */

	/*	Calcular el error del segundo lazo restando el valor de la velocidad determinada en la iteración anterior */

	/*	Introducción de la ecuación del controlador */

	/*	Calcular el valor de la velocidad para la iteración siguiente mediante el predictor de Smith
		Se puede implementar un filtro para comparar con la señal real*/

	/*	Determinar la posición actual integrando o leyendo un encoder absoluto.*/

	/*	Actualizar los valores de todos los vactores para la siguiente iteración*/
			// Variable para determinar el sentido de giro 
	
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+ 
	*/

	}
}


void CSteerControllerLowLevel::status_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Status Mode: %s", msg->data ? "true":"false" );
}

void CSteerControllerLowLevel::eje_x_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Steer_axis %.02f", msg->data );
}

void CSteerControllerLowLevel::eje_y_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Voltage pedal %.02f", msg->data );
}

void CSteerControllerLowLevel::GPIO7_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Reverse throttle direcction : %s", msg->data ? "true":"false" );
}