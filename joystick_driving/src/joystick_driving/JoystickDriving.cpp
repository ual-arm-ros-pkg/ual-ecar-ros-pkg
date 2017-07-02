/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <joystick_driving/JoystickDriving.h>
#include <ros/console.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

int pwm_steering_const = 0;
float eje_x = 0;
float eje_y = 0;

bool JoystickDriving::initialize()
{
	// Subscriber:
	MRPT_TODO("Sub to enable_joystick topic");
	ROS_INFO("JoystickDriving::initialize() ok.");

	m_pub_rev_relay     = m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_eje_x			= m_nh.advertise<std_msgs::Float64>("joystick_eje_x", 10);
	m_pub_eje_y			= m_nh.advertise<std_msgs::Float64>("joystick_eje_y", 10);
	m_pub_contr_status	= m_nh.advertise<std_msgs::Bool>("steer_controller_status", 10);

	{
		std_msgs::Bool b;
		b.data = true;
		m_pub_rev_relay.publish(b);
		m_pub_contr_status.publish(b);
	}

	{
		std_msgs::Float64 msg_f;
		msg_f.data = 1.0;
		m_pub_eje_x.publish(msg_f);
		m_pub_eje_x.publish(msg_f);
	}

}

bool JoystickDriving::iterate()
{
	float x,y,z,aux_s,aux_r;
	vector<bool> buttons;
	bool rev, control_mode;

	bool ok = m_joy.getJoystickPosition(0, x,y,z, buttons);
	if (!ok) {
		ROS_ERROR("JoystickDriving: Error reading Joystick!");
		return false;
	}
	const bool mode_btn = (buttons.size()>=7 && !buttons[6]);
	{
		std_msgs::Bool b;
		b.data = mode_btn;
		control_mode = mode_btn;
		m_pub_contr_status.publish(b);
	}

		// Eje X
		{
		std_msgs::Float64 msg_f;
		//  Aumento de resolucion
		if (buttons[4]) {
			aux_s = x * 0.2;
		}
		else {
			aux_s = x * 0.5;
		}
			
		if (buttons[1]) {
			eje_x = eje_x + aux_s;
			aux_s = eje_x;
		}
		else {
			aux_s = eje_x + aux_s;
		}

		msg_f.data = aux_s;
		m_pub_eje_x.publish(msg_f);
	}
	// Eje y
	{
		//  Aumento de resolucion
		if (buttons[5]) {
			aux_r =(float)((-y) * 0.2);
		}
		else {
			aux_r =(float)((-y) * 0.5);
		}
		
		if (buttons[0]) {
			eje_y = eje_y + aux_r;
			aux_r = eje_y;
		}
		else {
			aux_r = eje_y + aux_r;
		}
		std_msgs::Float64 msg_f;
		msg_f.data = eje_y;
		
		m_pub_eje_y.publish(msg_f);

	}
	// Rev button:
	// ---------------
	const bool reverse_btn = (buttons.size()>=4 && !buttons[3]);
	{
		std_msgs::Bool b;
		b.data = reverse_btn;
		m_pub_rev_relay.publish(b);
	}

	ROS_INFO("Joy: x:%f y:%f B3=%i B6=%i", aux_s,aux_r, buttons[3] ? 1:0, buttons[6] ? 1:0);

	return true;
}
