/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

class CSteerControllerLowLevel
{
public:

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh = ros::NodeHandle();
	ros::NodeHandle m_nh_params = ros::NodeHandle("~");

	ros::Publisher  m_pub_rev_relay, m_pub_pwm_steering, m_pub_voltage_pedal,m_pub_rev_steering;
/*	m_pub_rev_relay		:: Activa relé de marcha.
	m_pub_pwm_steering	:: Señal PWM para la dirección
	m_pub_voltage_pedal	:: Señal simulada del pedal de aceleración
	m_pub_rev_steering	:: Activa el Pin DIR del pololu
	m_pub_contr_status	:: Establece el modo de funcionamiento del controlador
*/
	ros::Subscriber m_sub_auto_pos, m_sub_pwm, m_sub_contr_status;
/*	Definir todas las variables que tiene que leer del joystick
*/

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

protected:
	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	double      m_steer_report_freq;
	mrpt::hwdrivers::CSerialPort m_serial;  //!< The serial COMMS object

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();
};
