/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |             Copyright (C) 2014-17  University of Almeria                  |
   +---------------------------------------------------------------------------+ */

#include <system_identification/SystemIdentification.h>
#include <thread>
#include <ros/console.h>
#include <functional>
#include <cstring>
#include <array>
#include <iostream>
#include <system_identification/Controller_parameters.h>
#include <system_identification/System_parameters.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

/* +------------------------+
   |		VARIABLES		|
   +------------------------+*/

/*AUXILIARES*/
float m_eje_x = 0;			/*Variable para la lectura del joystick derecho del mando*/
float m_eje_y = 0;			/*Variable para la lectura del joystick izquierdo del mando*/
float Tm = 0.05;			/*Tiempo entre iteraciones en segundos*/
double m_Encoder_Absoluto = 0;		/*Variable para la lectura del encoder absoluto*/
double m_enc_inc = 0;			/*Valor actual del encoder incremental*/
int m_pwm_steering = 0;
double m_dac_pedal = 0;
const int params_number = 3;
/*ESTIMADOR*/


SystemIdentification::SystemIdentification() :
	mrpt::utils::COutputLogger("SystemIdentification"),
	m_nh_params("~")
{
}

SystemIdentification::~SystemIdentification()
{
}

bool SystemIdentification::initialize()
{
	ROS_INFO("SystemIdentification::inicialize() ok.");

	m_pub_steer_controller_pos	= m_nh.advertise<system_identification::Controller_parameters>("steer_controller_pos", 10);
	m_pub_steer_controller_speed	= m_nh.advertise<system_identification::Controller_parameters>("steer_controller_speed", 10);
	m_pub_steer_systemparameters	= m_nh.advertise<system_identification::System_parameters>("steer_system_parameters", 10);

	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &SystemIdentification::ejexCallback, this);
	m_sub_eje_y		= m_nh.subscribe("joystick_eje_y", 10, &SystemIdentification::ejeyCallback, this);
	m_sub_pwm_steering	= m_nh.subscribe("arduino_daq_pwm6", 10, &SystemIdentification::PWMCallback, this);  	/*CHANGE*/
	m_sub_voltage_pedal	= m_nh.subscribe("arduino_daq_dac0", 10, &SystemIdentification::DACCallback, this);	/*CHANGE*/

	// Inicialization
	{
		system_identification::Controller_parameters msg_fp;
		msg_fp.controller_values.resize(params_number);
		for (int i = 0; i < 2; i++)
			msg_fp.controller_values[i] = m_q_steer_ext[i];
		m_pub_steer_controller_pos.publish(msg_fp);
		m_pub_steer_controller_speed.publish(msg_fp);
	}
	{
		system_identification::System_parameters msg_fsys;
		msg_fsys.system_values_a.resize(params_number);
		msg_fsys.system_values_b.resize(params_number);
		for (int i = 0; i < 2; i++)
		{
			msg_fsys.system_values_a[i] = m_steer_a[i];
			msg_fsys.system_values_b[i] = m_steer_b[i];
		}
		m_pub_steer_systemparameters.publish(msg_fsys);
	}
}

bool SystemIdentification::iterate()
{

	//TO-DO: Identification
		m_q_steer_ext[0]	= 2.9082;
		m_q_steer_ext[1]	= - 2.8061;
		m_q_steer_ext[2]	= 0;
		m_q_steer_int[0]	= - 0.4542;
		m_q_steer_int[1]	= - 0.0281;
		m_q_steer_int[2]	= 0;
		m_steer_b[0]		= 0.8291;
		m_steer_b[1]		= 0;
		m_steer_b[2]		= 0;
		m_steer_a[0]		= 1;
		m_steer_a[1]		= -0.1709;
		m_steer_a[0]		= 0;

//	ROS_INFO_COND_NAMED( m_Encoder_Abs[0] !=  m_Encoder_Abs[1], " test only " , "Encoder_Abs: %f ", m_Encoder_Abs[0]);



	// PUB
	system_identification::Controller_parameters msg_fp;
	msg_fp.controller_values.resize(params_number);
	for (int i = 0; i < 2; i++)
		msg_fp.controller_values[i] = m_q_steer_ext[i];
	m_pub_steer_controller_pos.publish(msg_fp);

	system_identification::Controller_parameters msg_fs;
	msg_fs.controller_values.resize(params_number);
	for (int i = 0; i < 2; i++)
		msg_fp.controller_values[i] = m_q_steer_int[i];
	m_pub_steer_controller_speed.publish(msg_fs);

	system_identification::System_parameters msg_fsys;
	msg_fsys.system_values_a.resize(params_number);
        msg_fsys.system_values_b.resize(params_number);
	for (int i = 0;i<2;i++)
	{
		msg_fsys.system_values_a[i] = m_steer_a[i];
		msg_fsys.system_values_b[i] = m_steer_b[i];
	}
	m_pub_steer_systemparameters.publish(msg_fsys);

}

// SUB
void SystemIdentification::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_eje_x = msg->data;
}

void SystemIdentification::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_eje_y = msg->data;
}

void SystemIdentification::PWMCallback(const std_msgs::UInt8::ConstPtr& msg)	/*CHANGE*/
{
	m_pwm_steering = msg->data;
}

void SystemIdentification::DACCallback(const std_msgs::Float64::ConstPtr& msg)	/*CHANGE*/
{
	m_dac_pedal = msg->data;
}
