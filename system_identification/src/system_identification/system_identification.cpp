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
double m_enc_abs = 0;		/*Variable para la lectura del encoder absoluto*/
double m_enc_inc = 0;		/*Valor actual del encoder incremental*/
float m_steer_adc = 0;
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

	m_pub_steer_controller_pos		= m_nh.advertise<system_identification::Controller_parameters>("steer_controller_pos", 10);
	m_pub_steer_controller_speed	= m_nh.advertise<system_identification::Controller_parameters>("steer_controller_speed", 10);
	m_pub_steer_systemparameters	= m_nh.advertise<system_identification::System_parameters>("steer_system_parameters", 10);

	m_sub_eje_x  			= m_nh.subscribe("joystick_eje_x", 10, &SystemIdentification::ejexCallback, this);
	m_sub_eje_y				= m_nh.subscribe("joystick_eje_y", 10, &SystemIdentification::ejeyCallback, this);
	m_sub_contr_status[0]	= m_nh.subscribe("vehicle_openloop_mode_steering", 10, &SystemIdentification::modeSteeringCallback, this);
	m_sub_contr_status[1]	= m_nh.subscribe("vehicle_openloop_mode_throttle", 10, &SystemIdentification::modeThrottleCallback, this);
	m_sub_autonomous_driving= m_nh.subscribe("vehicle_autonomous_mode", 10, &SystemIdentification::autonomousModeCallback, this);
	m_sub_Control_signal	= m_nh.subscribe("claraquino_control_signal", 10, &SystemIdentification::ControlCallback, this);
	m_sub_encoder			= m_nh.subscribe("claraquino_encoders", 10, &SystemIdentification::ENCCallback, this);
	m_sub_encoder_abs		= m_nh.subscribe("claraquino_abs_encoder", 10, &SystemIdentification::ENCAbsCallback, this);
	m_sub_steer_adc			= m_nh.subscribe("claraquino_adc", 10, &SystemIdentification::ADCCallback, this);

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
	//

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

void SystemIdentification::ControlCallback(const ual_ecar_vehicle_controller::ControlSignalSteer::ConstPtr& msg)
{
//	m_dac_pedal		= msg->Throttle_controller_signal;
	m_pwm_steering	= msg->Steer_controller_signal;
}

void SystemIdentification::ENCCallback(const ual_ecar_vehicle_controller::EncodersReading::ConstPtr& msg)
{
	m_enc_inc = msg->encoder_values[0];
}
void SystemIdentification::ENCAbsCallback(const ual_ecar_vehicle_controller::EncoderAbsReading::ConstPtr& msg)
{
	m_enc_abs = msg->encoder_value;
}
void SystemIdentification::ADCCallback(const ual_ecar_vehicle_controller::AnalogReading::ConstPtr& msg)
{
	m_steer_adc = msg->adc_data[0];
}
void SystemIdentification::autonomousModeCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_autonomous_driving_mode = msg->data;
}

void SystemIdentification::modeSteeringCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_mode_openloop_steer = msg->data;
}
void SystemIdentification::modeThrottleCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_mode_openloop_throttle = msg->data;
}