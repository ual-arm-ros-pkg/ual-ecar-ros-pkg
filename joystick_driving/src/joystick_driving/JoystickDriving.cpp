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
	float x,y,z;
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
		if (buttons[4])
			aux_s[0] = z * 0.2;
		else
			aux_s[0] = z * 0.5;
		// Saturacion
		if (eje_x + aux_s[0] > 1)
			aux_s[0] = 1 - eje_x;
		if (eje_x + aux_s[0] < -1)
			aux_s[0] = -1 - eje_x;
		// Cambio del valor central
		if (buttons[1]) {
			eje_x = eje_x + aux_s[0];
			aux_s[0] = eje_x;
		}
		else 
			aux_s[0] = eje_x + aux_s[0];

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
		// Cambio del valor central
		if (buttons[0]) {
			eje_y = eje_y + aux_r[0];
			aux_r[0] = eje_y;
		}
		else 
			aux_r[0] = eje_y + aux_r[0];

		std_msgs::Float64 msg_f;
		msg_f.data = aux_r[0];
		
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

	// ROS_INFO("Joy: x:%f y:%f B3=%i B6=%i", aux_s,aux_r, buttons[3] ? 1:0, buttons[6] ? 1:0);
	ROS_INFO_COND_NAMED( aux_s[0] !=  aux_s[1] || aux_r[0] !=  aux_r[1] || rev_btn != reverse_btn || mode_btn != control_btn, " test only " ,"Joy: x:%f y:%f B3=%i B6=%i", aux_s[0],aux_r[0], buttons[3] ? 1:0, buttons[6] ? 1:0);
	aux_s[1] = aux_s[0];
	aux_r[1] = aux_r[0];
	rev_btn = reverse_btn;
	control_btn = mode_btn;

	return true;
}
