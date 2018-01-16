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
#include <ual_ecar_vehicle_controller/AnalogReading.h>
#include <ual_ecar_vehicle_controller/ControlSignal.h>
#include <ual_ecar_vehicle_controller/EncoderAbsReading.h>
#include <ual_ecar_vehicle_controller/EncodersReading.h>
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

	ros::Publisher  m_pub_steer_controller_pos,m_pub_steer_controller_speed, m_pub_steer_systemparameters;
/*	m_pub_steer_controller_pos	::	Parametros del controlador definido para la posicion de la dirección.
	m_pub_steer_controller_pos	::	Parametros del controlador definido para la velocidad de la dirección.
	m_pub_steer_systemparameters	::	Parámetros obtenidos de la identificación del sistema.
*/
	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_encoder, m_sub_encoder_abs, m_sub_Control_signal,m_sub_steer_adc,m_sub_contr_status[2], m_sub_autonomous_driving;
/*	m_sub_eje_x 		::	Lectura de datos del joystick izq. Eje horizontal. Dirección
	m_sub_eje_y		::	Lectura de datos del joystick izq. Eje vertical. Aceleración
	m_sub_encoder		::	...
	m_sub_encoder_abs	::	Lectura del encoder absoluto
*/

	bool initialize();

	/** called when work is to be done */
	bool iterate();

	protected:

	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
	void ControlCallback(const ual_ecar_vehicle_controller::ControlSignal::ConstPtr& msg);
	void ENCCallback(const ual_ecar_vehicle_controller::EncodersReading::ConstPtr& msg);
	void ENCAbsCallback(const ual_ecar_vehicle_controller::EncoderAbsReading::ConstPtr& msg);
	void ADCCallback(const ual_ecar_vehicle_controller::AnalogReading::ConstPtr& msg);
	void autonomousModeCallback(const std_msgs::Bool::ConstPtr &msg);
	void modeSteeringCallback(const std_msgs::Bool::ConstPtr &msg);
	void modeThrottleCallback(const std_msgs::Bool::ConstPtr &msg);

	bool m_autonomous_driving_mode{false};
	bool m_mode_openloop_steer{true};
	bool m_mode_openloop_throttle{true};
	float m_q_steer_ext[3]	= {0.0,0.0,0.0};
	float m_q_steer_int[3]	= {0.0,0.0,0.0};
	float m_steer_b[3]		= {0.0,0.0,0.0};
	float m_steer_a[3]		= {1.0,0.0,0.0};
};
