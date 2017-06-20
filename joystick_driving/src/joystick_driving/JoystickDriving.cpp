/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <joystick_driving/JoystickDriving.h>
#include <ros/console.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

bool JoystickDriving::initialize()
{
	// Subscriber:
	MRPT_TODO("Sub to enable_joystick topic");
	ROS_INFO("JoystickDriving::initialize() ok.");

	m_pub_rev_relay     = m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_pwm_steering  = m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm3", 10);
	m_pub_voltage_pedal = m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);


	{
		std_msgs::Bool b;
		b.data = true;
		m_pub_rev_relay.publish(b);
	}

	{
		std_msgs::UInt8 msg_ui;
		msg_f.data = 0;
		m_pub_pwm_steering.publish(msg_f);
	}

	{
		std_msgs::Float64 msg_f;
		msg_f.data = 1.0;
		m_pub_voltage_pedal.publish(msg_f);
	}

}

bool JoystickDriving::iterate()
{
	float x,y,z;
	int aux;
	vector<bool> buttons;

	bool ok = m_joy.getJoystickPosition(0, x,y,z, buttons);
	if (!ok) {
		ROS_ERROR("JoystickDriving: Error reading Joystick!");
		return false;
	}

	// Rev button:
	// ---------------
	const bool reverse_btn = (buttons.size()>=4 && !buttons[3]);
	{
		std_msgs::Bool b;
		b.data = reverse_btn;
		m_pub_rev_relay.publish(b);
	}

	// PWM steering:
	// ----------------
	// [-1,1] -> [-1,1]
	{
		std_msgs::UInt8 msg_i;
		aux = ((int) * 125) + 125; 
		msg_i.data = aux;
		m_pub_pwm_steering.publish(msg_i);
	}

	// Volt pedal:
	// ------------
	// |[-0.8,0]| -> [0,5]
	if (y<=0)
	{
		// Accel:
		const double K = 4.0/0.85;
		const double volt_pedal = 1.0+(-y)*K;

		std_msgs::Float64 msg_f;
		msg_f.data = volt_pedal;
		ROS_INFO("Pedal: %.02f volts", volt_pedal);

		m_pub_voltage_pedal.publish(msg_f);
	}
	else
	{
		// Brake:
		// NOT IMPLEMENTED YET!!
	}

  ROS_INFO("Joy: x:%f y:%f z:%f nbut=%u B3=%i", x,y,z, (unsigned int)buttons.size(), buttons[3] ? 1:0 );

	return true;
}
