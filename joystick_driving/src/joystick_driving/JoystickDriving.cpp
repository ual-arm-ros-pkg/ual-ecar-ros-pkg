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

int pwm_steering_const = 0;
float steer_ref = 0;

bool JoystickDriving::initialize()
{
	// Subscriber:
	MRPT_TODO("Sub to enable_joystick topic");
	ROS_INFO("JoystickDriving::initialize() ok.");

	m_pub_rev_relay     = m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_rev_steering  = m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output4", 10);
	m_pub_pwm_steering  = m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm3", 10);
	m_pub_voltage_pedal = m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);
	m_pub_contr_status  = m_nh.advertise<std_msgs::Bool>("steer_controller_status", 10);
	m_pub_steer_ref     = m_nh.advertise<std_msgs::Float64>("steer_controller_ref", 10);
	m_pub_speed_ref     = m_nh.advertise<std_msgs::Float64>("steer_controller_speed_ref", 10);

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
		m_pub_speed_ref.publish(msg_f);
		m_pub_steer_ref.publish(msg_f);
	}

}

bool JoystickDriving::iterate()
{
	float x,y,z,aux_s;
	int aux;
	vector<bool> buttons;
	bool rev, control_mode;
	float Sat = 254;

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
	if (!control_mode)
	{	
		ROS_INFO("Controlador eCAR en modo automatico");
		// Steer_ref:
		// ----------------
		{
			std_msgs::Float 64 msg_f;
			//  Aumento de resolucion
			if (buttons[4]) {
				aux_s =x * 10;
			}
			else {
				aux_s = x * 40;
			}
			//	Saturacion
			if ((aux_s + steer_ref) < -Sat) {
				aux_s =   0;
				steer_ref = -Sat;
			}
			if ((aux_s + steer_ref) > Sat) {
				aux_s = 0;
				steer_ref = 254;
			}
			
			if (buttons[1]) {
				steer_ref = steer_ref + aux_s;
				aux_s = steer_ref;
			}
			else {
				aux_s = steer_ref + aux_s;
			}
			ROS_INFO("Steer reference: %f ", aux_s);

			msg_f.data = aux_s;
			m_pub_steer_ref.publish(msg_f);
		}
		return true;
	} 
	else
	{

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
		std_msgs::UInt8 msg_ui;
	//  	Aumento de resolucion
		if (buttons[4])
		{
			aux = (int)(x * 10);
		}
		else
		{
			aux = (int)(x * 40);
		}
	//	Saturacion
		if ((aux + pwm_steering_const) < -254)
		{
			aux =   0;
			pwm_steering_const = -254;
		}
		if ((aux + pwm_steering_const)>254) 
		{
			aux = 0;
			pwm_steering_const = 254;
		}
		
		if (buttons[1])
		{
			pwm_steering_const = pwm_steering_const + aux;
			aux = pwm_steering_const;
		}
		else
		{
			aux = pwm_steering_const + aux;
		}
		ROS_INFO("PWM: %i ", aux);
		if (aux < 0)
		{
			aux = -aux;
			rev = false;
		}
		else 
		{
			rev = true;
		}
		msg_ui.data = aux;
		m_pub_pwm_steering.publish(msg_ui);
	}

	const bool reverse_steering_btn = (rev);
	{
		std_msgs::Bool msg_b;
		msg_b.data = reverse_steering_btn;
		m_pub_rev_steering.publish(msg_b);
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
}
