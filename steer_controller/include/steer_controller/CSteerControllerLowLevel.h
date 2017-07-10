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
#include <mrpt/utils/COutputLogger.h>
#include <arduino_daq/EncodersReading.h>
#include <functional>


class CSteerControllerLowLevel : public mrpt::utils::COutputLogger
{
public:
	CSteerControllerLowLevel();
	virtual ~CSteerControllerLowLevel();
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;// = ros::NodeHandle();
	ros::NodeHandle m_nh_params;// = ros::NodeHandle("~");

	ros::Publisher  m_pub_rev_relay, m_pub_pwm_steering, m_pub_voltage_pedal,m_pub_rev_steering;
/*	m_pub_rev_relay		::	Activa relé de marcha.
	m_pub_pwm_steering	::	Señal PWM para la dirección
	m_pub_voltage_pedal	::	Señal simulada del pedal de aceleración
	m_pub_rev_steering	::	Activa el Pin DIR del pololu
	m_pub_contr_status	::	Establece el modo de funcionamiento del controlador
*/
	ros::Subscriber m_sub_rev_relay, m_sub_eje_x, m_sub_eje_y, m_sub_contr_status, m_sub_encoder;
/*	m_sub_rev_relay		::	Activa relé de marcha desde el control automático
	m_sub_eje_x			::	Lectura de datos del joystick izq. Eje horizontal. Dirección
	m_sub_eje_y			::	Lectura de datos del joystick izq. Eje vertical. Aceleración
	m_sub_contr_status	::	Booleano para la determinación si se encuentra el control en modo manual o automático
*/

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

	protected:

	void statusCallback(const std_msgs::Bool::ConstPtr& msg);
	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
	void GPIO7Callback(const std_msgs::Bool::ConstPtr& msg);
	void encoderCallback(const arduino_daq::EncodersReading::ConstPtr& msg);

	double m_ep[3] = {0,0,0};
	double m_up[6] = {0,0,0,0,0,0};
	double m_es[2] = {0,0};
	double m_yp[3] = {0,0,0};
	double m_ys[4] = {0,0,0,0};
	double m_Encoder[2] = {0,0};
	int m_us[5] = {0,0,0,0,0};

};
