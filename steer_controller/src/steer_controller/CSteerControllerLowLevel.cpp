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
bool Status_mode;
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
	m_pub_pwm_steering	= m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm6", 10);
	m_pub_voltage_pedal	= m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);

	m_sub_contr_status	= m_nh.subscribe("steer_controller_status", 10, &CSteerControllerLowLevel::statusCallback, this);
	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &CSteerControllerLowLevel::ejexCallback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &CSteerControllerLowLevel::ejeyCallback, this);
	m_sub_rev_relay		= m_nh.subscribe("arduino_daq_GPIO_output7",10, &CSteerControllerLowLevel::GPIO7Callback, this);
	m_sub_encoder		= m_nh.subscribe("arduino_daq_encoders", 10, &CSteerControllerLowLevel::encoderCallback, this);

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
	int pwm_steering;
	double voltaje_pedal,rpm;
	bool b2,b1,b3;

	// Lectura del modo de control
	bool ok = Status_mode;

	std_msgs::UInt8 msg_ui;
	std_msgs::Float64 msg_f;
	std_msgs::Bool msg_b;

	/*Lectura del encoder de la direccion y predictor de smith de la velocidad*/
	rpm = (m_Encoder[0] - m_Encoder[1]) / 0.05;
	m_ys[0] = m_ys[1] * 0.1709 - 0.0775 * m_us[1+3];

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
		pwm_steering = (int)(Eje_x * 254);
		if (pwm_steering < 0)
		{
			pwm_steering = - pwm_steering;
			msg_b.data = false;
		}
		else
		{
			msg_b.data = true;
		}
		m_us[0] = pwm_steering;
		msg_ui.data = pwm_steering;
		m_pub_rev_steering.publish(msg_b);
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", msg_ui.data);
		ROS_INFO("Encoder: %f ", m_Encoder[0]);
		// m_R_steer_f[0] = m_Encoder[0]; // Para implementar mecanismo de transferencia sin salto

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
		double m_R_steer_f[0] = (double)(Eje_x * 40);

	/*	Filtro en la referencia para disminuir la sobreoscilación en la señal de salida */
		m_R_steer[0] = 0.9649 * m_R_steer[1] + 0.0351 * m_R_steer_f[0];

	/*	Saturación de la referencia para protección contra sobrecorrientes*/
	/*	double sat_ref = 4.55;
		double pendiente = (m_R_steer[0] - m_R_steer[1]) / 0.05;
		if (pendiente >= sat_ref)
		{
			m_R_steer[0] = (m_R_steer[1] + sat_ref);
		}
	*/
	/*	Corrección del sentido de las ruedas*/
		m_R_steer[0] = - m_R_steer[0];

		ROS_INFO("Referencia: %f ", m_R_steer[0]);
		ROS_INFO("Encoder: %f ", m_Encoder[0]);

	/*	Calculo del error al restar la restar el encoder a la referencia de posicion */
		m_ep[0] = m_R_steer[0] - m_Encoder[0];

	/*	Controlador lazo externo */
		m_up[0] = m_up[1] + 2.9082 * m_ep[0] - 2.8061 * m_ep[1];

	/*	Mecanismo Anti-windup para protección*/

	/*	Calcular el error del segundo lazo restando el valor de la velocidad determinada en la iteracion anterior */
		m_es[0] = m_up[0] - m_ys[0];/* - (rpm - m_ys[3]);*/

	/*	Introduccion de la ecuacion del controlador */
		m_us[0] = (int)(m_us[1] - 2.8261 * m_es[0] - 0.1750 * m_es[1]);

	/*	Implementar saturacion */
		if (m_us[0] > 254)
		{
			m_us[0] = 254;
		}
		if (m_us[0] < -254)
		{
			m_us[0] = -254;
		}
		ROS_INFO("Yp: %f, Encoder: %f, Ep: %f, Up: %f, Ys: %f, Es: %f, Us: %i", m_yp[0],rpm, m_ep[0], m_up[0],m_ys[0],m_es[0],m_us[0]);
	
	/*	Implementar protección que detecte que el encoder está en el límite y solo permita girar en el sentido contrario*/

	//	..............

	/*	Envio de datos a los parametros correspondientes de ROS*/
		if (m_us[0] < 0)
		{
			msg_ui.data = - m_us[0];
			msg_b.data = false;
		}
		else
		{
			msg_ui.data = m_us[0];
			msg_b.data = true;
		}
		m_pub_rev_steering.publish(msg_b);
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", msg_ui.data);

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
		// Bool
		
		b1 = GPIO7;
		msg_b.data = GPIO7;
		m_pub_rev_relay.publish(msg_b);

		ROS_INFO("Rev Relay = %s", b1 ? "true":"false");

	}
	/* Actualizacion de valores*/
	m_R_steer[1] = m_R_steer[0];
	m_R_steer_f[1] = m_R_steer_f[0];
	for (int i=2;i>=1;i--)
	{
		m_yp[i] = m_yp[i-1];
		m_ep[i] = m_ep[i-1];
	}
	for (int i=5;i>=1;i--)
	{
		m_up[i] = m_up[i-1];
	}
	m_es[1] = m_es[0];
	m_Encoder[1] = m_Encoder[0];
	for (int i=3;i>=1;i--)
	{
		m_ys[i] = m_ys[i-1];
	}

	for (int i=4;i>=1;i--)
	{
		m_us[i] = m_us[i-1];
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

void CSteerControllerLowLevel::encoderCallback(const arduino_daq::EncodersReading::ConstPtr& msg)
{
	m_Encoder[0] = - (msg->encoder_values[0]) / 1824.9;
	// m_Enc_motor[0] = (msg->encoder_values[1]);
}
