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
float eje_x[2] = {0,0};
float eje_y[2] = {0,0};
bool button[3] = {0,0,0};


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
	const bool steer_btn = (buttons.size()>7 && !buttons[7]);
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
	const bool autonomous_btn = (buttons.size()>8 && !buttons[8]);
	{
		std_msgs::Bool c;
		c.data = autonomous_btn;
		m_pub_autonomous_driving.publish(c);
	}

	std_msgs::Float64 msg_f;
	
	// Axis X
	{
		//  Resolution increase
		if (buttons[4])
			eje_x[0] = z * 0.2;
		else
			eje_x[0] = z * 0.5;
		// Saturation
		if (eje_x[0] > 1)
			eje_x[0] = 1;
		if (eje_x[0] < -1)
			eje_x[0] = -1;

		msg_f.data = eje_x[0];
		m_pub_eje_x.publish(msg_f);
	}
	// Axis y
	{
		//  Resolution increase
		if (buttons[5])
			eje_y[0] =(float)((-y) * 0.2);
		else
			eje_y[0] =(float)((-y) * 0.5);
		// Saturacion
		if (eje_y[0] > 1)
			eje_y[0] = 1;
		if (eje_y[0] < -1)
			eje_y[0] = - 1;

		msg_f.data = eje_y[0];
		m_pub_eje_y.publish(msg_f);
	}

	if(eje_x[0] != eje_x[1] || eje_y[0] != eje_y[1] || steer_btn != button[0] || throttle_btn != button[1] || autonomous_btn != button[2])
		ROS_INFO("Joy: x:%f y:%f steer_btn=%i throttle_btn=%i autonomous_btn=%i", eje_x[0],eje_y[0], steer_btn ? 1:0, throttle_btn ? 1:0, autonomous_btn);

	button[0] = steer_btn;
	button[1] = throttle_btn;
	button[2] = autonomous_btn;
	eje_x[1] = eje_x[0];
	eje_y[1] = eje_y[0];

	return true;
}
