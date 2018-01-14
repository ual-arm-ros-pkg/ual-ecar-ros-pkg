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
#include <functional>


class SystemIdentification : public mrpt::utils::COutputLogger
{
public:
	SystemIdentification();
	virtual ~SystemIdentification();
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;// = ros::NodeHandle();
	ros::NodeHandle m_nh_params;// = ros::NodeHandle("~");

	ros::Publisher  m_pub_controller_pos, m_pub_controller_speed, m_pub_systemparameters;
/*	m_pub_controller		::	Parametros del controlador definido para la posicion.
	m_pub_controller_speed	::	Parametros del controlador definido para la velocidad.
	m_pub_systemparameters	::	Parámetros obtenidos de la identificación del sistema.
*/
	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_encoder, m_sub_encoder_abs, m_sub_pwm_steering, m_sub_voltage_pedal;
/*	m_sub_rev_relay		::	Activa relé de marcha desde el control automático
	m_sub_eje_x			::	Lectura de datos del joystick izq. Eje horizontal. Dirección
	m_sub_eje_y			::	Lectura de datos del joystick izq. Eje vertical. Aceleración
	m_sub_encoder		::	...
	m_sub_encoder_abs	::	Lectura del encoder absoluto
*/

	bool initialize();

	/** called when work is to be done */
	bool iterate();

	protected:

	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
	void PWMCallback(const std_msgs::UInt8::ConstPtr& msg);
	void DACCallback(const std_msgs::Float64::ConstPtr& msg);

	double m_q_ext[3] = {0,0,0};
	double m_q_int[3] = {0,0,0};

};
