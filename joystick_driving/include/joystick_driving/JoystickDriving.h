/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <ros/ros.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

class JoystickDriving
{
public:
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh = ros::NodeHandle();
	ros::NodeHandle m_nh_params = ros::NodeHandle("~");

	ros::Subscriber m_sub_enable_joystick;
	ros::Publisher  m_pub_rev_relay, m_pub_pwm_steering, m_pub_voltage_pedal,m_pub_rev_steering;

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

protected:
	// Local class members:
	mrpt::hwdrivers::CJoystick m_joy;



};
