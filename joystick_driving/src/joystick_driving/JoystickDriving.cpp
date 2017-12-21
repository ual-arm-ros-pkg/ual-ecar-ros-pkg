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
bool rev_btn = false;
bool control_btn = true;


bool JoystickDriving::initialize()
{
	// Subscriber:
	MRPT_TODO("Sub to enable_joystick topic");
	ROS_INFO("JoystickDriving::initialize() ok.");

	m_pub_eje_x			= m_nh.advertise<std_msgs::Float64>("joystick_eje_x", 10);
	m_pub_eje_y			= m_nh.advertise<std_msgs::Float64>("joystick_eje_y", 10);
	m_pub_contr_status[0]		= m_nh.advertise<std_msgs::Bool>("vehicle_openloop_mode_steering", 10);
	m_pub_contr_status[1]		= m_nh.advertise<std_msgs::Bool>("vehicle_openloop_mode_throttle", 10);
	m_pub_autonomous_driving	= m_nh.advertise<std_msgs::Bool>("vehicle_autonomous_mode", 10);

	{
		std_msgs::Bool b;
		b.data = true;
		m_pub_contr_status[0].publish(b);
		m_pub_contr_status[1].publish(b);
		b.data = false;
		m_pub_autonomous_driving.publish(b);
	}

	{
		std_msgs::Float64 msg_f;
		msg_f.data = 0.0;
		m_pub_eje_x.publish(msg_f);
		m_pub_eje_y.publish(msg_f);
	}

}

bool JoystickDriving::iterate()
{
	float x,y,z;
	vector<bool> buttons;

	bool ok = m_joy.getJoystickPosition(0, x,y,z, buttons);
	if (!ok) {
		ROS_ERROR("JoystickDriving: Error reading Joystick!");
		return false;
	}
	const bool steer_btn = (buttons.size()>8 && !buttons[8]);
	{
		std_msgs::Bool a;
		a.data = steer_btn;
		m_pub_contr_status[0].publish(a);
	}
	const bool throttle_btn = (buttons.size()>6 && !buttons[6]);
	{
		std_msgs::Bool b;
		b.data = throttle_btn;
		m_pub_contr_status[1].publish(b);
	}
	const bool autonomous_btn = (buttons.size()>7 && !buttons[7]);
	{
		std_msgs::Bool c;
		c.data = mode_btn;
		m_pub_autonomous_driving.publish(c);
	}

		// Eje X
		{
		std_msgs::Float64 msg_f;
		//  Aumento de resolucion
		if (buttons[4])
			aux_s[0] = z * 0.2;
		else
			aux_s[0] = z * 0.5;
		// Saturacion
		if (eje_x + aux_s[0] > 1)
			aux_s[0] = 1 - eje_x;
		if (eje_x + aux_s[0] < -1)
			aux_s[0] = -1 - eje_x;

		msg_f.data = aux_s[0];
		m_pub_eje_x.publish(msg_f);
	}
	// Eje y
	{
		//  Aumento de resolucion
		if (buttons[5])
			aux_r[0] =(float)((-y) * 0.2);
		else
			aux_r[0] =(float)((-y) * 0.5);
		// Saturacion
		if (eje_y + aux_r[0] > 1)
			aux_r[0] = 1 - eje_y;
		if (eje_y + aux_r[0] < 0)
			aux_r[0] = - eje_y;

		std_msgs::Float64 msg_f;
		msg_f.data = aux_r[0];

		m_pub_eje_y.publish(msg_f);
	}

	ROS_INFO("Joy: x:%f y:%f steer_btn=%i throttle_btn=%i autonomous_btn=%i", aux_s[0],aux_r[0], buttons[6] ? 1:0, buttons[7] ? 1:0, buttons[8] ? 1:0);

	return true;
}
