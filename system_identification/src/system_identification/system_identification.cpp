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
double m_Encoder_Absoluto;	/*Variable para la lectura del encoder absoluto*/
double m_enc_inc = 0;		/*Valor actual del encoder incremental*/
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

	m_pub_controller_pos	= m_nh.advertise<std::vector<double>>("controller_pos", 10);
	m_pub_controller_speed	= m_nh.advertise<system_identification::Controller_parameters>("controller_speed", 10);
	m_pub_systemparameters	= m_nh.advertise<system_identification::System_parameters>("system_parameters", 10);

	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &SystemIdentification::ejexCallback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &SystemIdentification::ejeyCallback, this);
	m_sub_encoder		= m_nh.subscribe("arduino_daq_encoders", 10, &SystemIdentification::encoderCallback, this);
	m_sub_encoder_abs	= m_nh.subscribe("arduino_daq_abs_encoder", 10, &SystemIdentification::encoderAbsCallback, this);
	m_sub_pwm_steering	= m_nh.subscribe("arduino_daq_pwm6", 10, &SystemIdentification::PWMCallback, this);
	m_sub_voltage_pedal	= m_nh.subscribe("arduino_daq_dac0", 10, &SystemIdentification::DACCallback, this);

	// Inicialization
	{
		std::vector<double> msg_f;
		msg_f.data[3] = {1.0, 1.0, 1.0};
		m_pub_controller_pos.publish(msg_f);
//		m_pub_controller_speed.publish(msg_f);
	}
/*	{
		system_identification::System_parameters msg_fsys;
		msg_fsys.data[3] = {1.0, 1.0, 1.0};
		m_pub_systemparameters.publish(msg_fsys);
	}
*/
}

bool SystemIdentification::iterate()
{

	//Pendiente de crear el nodo correpondiente que comunique estos parametros
		m_q_ext[0] = 1.8903;
		m_q_ext[1] = - 1.8240;
		m_q_ext[2] = 0;
		m_q_int[0] = - 2.85;
		m_q_int[1] = - 0.1765;
		m_q_int[2] = 0;


	std::vector<double> msg_fp;
/*	system_identification::Controller_parameters msg_fs;
	system_identification::System_parameters msg_fsys;
	
	ROS_INFO_COND_NAMED( m_Encoder_Abs[0] !=  m_Encoder_Abs[1], " test only " , "Encoder_Abs: %f ", m_Encoder_Abs[0]);

	msg_fp.data = - m_q_ext;
	msg_fs.data = m_q_int;
	msg_fsys.data = false;
	m_pub_controller_pos.publish(msg_fp);
	m_pub_controller_speed.publish(msg_fs);
	m_pub_systemparameters.publish(msg_fsys);
*/
	msg_fp.data = - m_q_ext;
	m_pub_controller_pos.publish(msg_fp);
}


void SystemIdentification::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_eje_x = msg->data;
}

void SystemIdentification::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_eje_y = msg->data;
}

void SystemIdentification::encoderCallback(const arduino_daq::EncodersReading::ConstPtr& msg)
{
	m_enc_inc = (msg->encoder_values[0]) / 1824.9; //( ppv * reductor * n vueltas ) / ang_max 
	//	m_Encoder_m[0] = (msg->encoder_values[1]);	// Comprobar valor del encoder
}

void SystemIdentification::encoderAbsCallback(const arduino_daq::EncoderAbsReading::ConstPtr& msg)
{
	m_Encoder_Absoluto = (msg->encoder_value);
}

void SystemIdentification::PWMCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	int m_pwm_steering = msg->data;
}

void SystemIdentification::DACCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double m_dac_pedal = msg->data;
}