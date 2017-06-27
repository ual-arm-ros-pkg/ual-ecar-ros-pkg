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
	m_sub_steer_ref  	= m_nh.subscribe("steer_controller_ref", 10, &CSteerControllerLowLevel::steer_ref_Callback, this);
	m_sub_speed_ref		= m_nh.subscribe("steer_controller_speed_ref", 10, &CSteerControllerLowLevel::speed_ref_Callback, this);
	m_sub_auto_pos		= m_nh.subscribe("steer_controller_auto_pos_enable",10, &CSteerControllerLowLevel::autoPosEnableCallback, this);
	m_sub_pwm			= m_nh.subscribe("arduino_daq_pwm3", 10, &CSteerControllerLowLevel::pwmCallback, this);
	m_sub_rev_relay		= m_nh.subscribe("arduino_daq_GPIO_output7", 10, &CSteerControllerLowLevel::GPIO7_Callback, this);
	m_sub_rev_steering	= m_nh.subscribe("arduino_daq_GPIO_output4", 10, &CSteerControllerLowLevel::GPIO4_Callback, this);
	m_sub_voltage_pedal	= m_nh.subscribe("arduino_daq_dac0", 10, &CSteerControllerLowLevel::throttle_Callback, this);

	// Inicialization
	{
		std_msgs::Bool b;
		b.data = true;
		m_pub_rev_relay.publish(b);
		m_pub_contr_status.publish(b);
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

	bool ok = m_sub_contr_status;

	if (!ok)
	{
	//ROS_ERROR("Controlador eCAR en modo manual");
	return true;
	}
	else
	{
	//ROS_INFO("Controlador eCAR en modo automatico");
	// PWM steering:
	// ----------------
	// [-1,1] -> [-1,1]
	{
	
	}
	// Rev steering button:
	// ---------------
	const bool reverse_steering_btn = (rev);
	{
		std_msgs::Bool msg_b;
		msg_b.data = reverse_steering_btn;
		m_pub_rev_steering.publish(msg_b);
	}
	// Volt pedal:
	// ------------
	// |[-0.8,0]| -> [0,5]
	{
	
	}
	// Rev button:
	// ---------------
	const bool reverse_btn = (/*Colocar variable del codigo del controlador*/);
	{
		std_msgs::Bool b;
		b.data = reverse_btn;
		m_pub_rev_relay.publish(b);
	}
	}
}
void CSteerControllerLowLevel::autoPosEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Autocontrol mode: %s", msg->data ? "true":"false" );
}

void CSteerControllerLowLevel::pwmCallback(const std_msgs::Uint8::ConstPtr& msg)
{
	ROS_INFO("PWM: %i", msg->data );
}

void CSteerControllerLowLevel::status_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Status Mode: %s", msg->data ? "true":"false" );
}

void CSteerControllerLowLevel::steer_ref_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Steer reference %.02f", msg->data );
}

void CSteerControllerLowLevel::speed_ref_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Speed reference %.02f", msg->data );
}

void CSteerControllerLowLevel::throttle_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Voltage pedal %.02f", msg->data );
}

void CSteerControllerLowLevel::GPIO7_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Reverse throttle direcction : %s", msg->data ? "true":"false" );
}

void CSteerControllerLowLevel::GPIO4_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Reverse steer direcction : %s", msg->data ? "true":"false" );
}