/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <steer_controller/CSteerControllerLowLevel.h>
#include <thread>
#include <ros/console.h>
#include <steer_controller/SteerControllerStatus.h>
#include <functional>
#include <cstring>
#include <array>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

/* +------------------------+
   |		VARIABLES		|
   +------------------------+*/

/*AUXILIARES*/
float Eje_x = 0;			/*Variable para la lectura del joystick derecho del mando*/
float Eje_y = 0;			/*Variable para la lectura del joystick izquierdo del mando*/
bool Status_mode;			/*Variable para la comprobacion del modo de control*/
bool GPIO7 = false;			/*Variable asosciada al rele de la marcha del vehículo*/
int dep = 0;				/*Variable auxiliar para mostrar mensaje del modo de control en consola*/
int lim = 0;				/*Variable auxiliar indicadora si el mecanismo se encuentra proximo al extremo y evitar que continue avanzando*/
double aux = 0;				/*Variable auxiliar indicadora de la posicion del encoder incremental en el momento de la recalibracion*/
double ang_inicial = 0;		/*Variable auxiliar indicadora de la posicion del encoder absoluto en el momento de la recalibracion*/
double m_enc_inc = 0;		/*Valor actual del encoder incremental*/
bool red = true;			/*Variable auxiliar indicadora si es necesaria la recalibracion*/

double pos_ant = 0;			/*Valor de la posicion en la iteracion anterior del encoder absoluto*/
bool paso = false;			/*Variable auxiliar indicadora de si el encoder absoluto ha pasado del valor 1024*/
double m_Encoder_Absoluto;

/*CONTROLADOR*/


/*PROTECCIONES*/


CSteerControllerLowLevel::CSteerControllerLowLevel() :
	mrpt::utils::COutputLogger("CSteerControllerLowLevel"),
	m_nh_params("~")
{
}

CSteerControllerLowLevel::~CSteerControllerLowLevel()
{
}

bool CSteerControllerLowLevel::initialize()
{
	ROS_INFO("CSteerControllerLowLevel::inicialize() ok.");

	m_pub_rev_relay		= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output7", 10);
	m_pub_rev_steering	= m_nh.advertise<std_msgs::Bool>("arduino_daq_GPIO_output4", 10);
	m_pub_pwm_steering	= m_nh.advertise<std_msgs::UInt8>("arduino_daq_pwm6", 10);
	m_pub_voltage_pedal	= m_nh.advertise<std_msgs::Float64>("arduino_daq_dac0", 10);

	m_sub_contr_status	= m_nh.subscribe("steer_controller_status", 10, &CSteerControllerLowLevel::statusCallback, this);
	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &CSteerControllerLowLevel::ejexCallback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &CSteerControllerLowLevel::ejeyCallback, this);
	m_sub_rev_relay		= m_nh.subscribe("arduino_daq_GPIO_output7",10, &CSteerControllerLowLevel::GPIO7Callback, this);
	m_sub_encoder		= m_nh.subscribe("arduino_daq_encoders", 10, &CSteerControllerLowLevel::encoderCallback, this);
	m_sub_encoder_abs	= m_nh.subscribe("arduino_daq_abs_encoder", 10, &CSteerControllerLowLevel::encoderAbsCallback, this);

	// Inicialization
	{
		std_msgs::Bool b;
		b.data = false;
		m_pub_rev_relay.publish(b);
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
	}



}

bool CSteerControllerLowLevel::iterate()
{
	double voltaje_pedal,rpm,encoder_value;
	bool b2,b3;
	int max_p = 50;		// Valor maximo que puede alcanzar la direccion

	// Lectura del modo de control
	bool Manual_Control = Status_mode;

	std_msgs::UInt8 msg_ui;
	std_msgs::Float64 msg_f;
	std_msgs::Bool msg_b;

	//Calibracion encoder Absoluto
		if(pos_ant == 0 && m_Encoder_Absoluto < 280)					/*Comprobacion por si el encoder se encuentra en una posicion superior a 1024*/
			paso = true;

		if(m_Encoder_Absoluto - pos_ant < - 500 || paso == true)		/*Comprobacion por si el encoder se inicia en una posicion superior a 1024 o*/
		{													/* se produce un salto durante la operacion desde 1024 a 0*/
			encoder_value = m_Encoder_Absoluto - 303 + 1024;/*Correccion del valor del encoder y del offset*/
			paso = true;									/*Variable que indica que el encoder opera en posiciones superiores a 1024*/
			if(m_Encoder_Absoluto + 1024 > 1400)
				paso = false;
		}
		if(paso == false)
			encoder_value = m_Encoder_Absoluto - 303;

		pos_ant = m_Encoder_Absoluto;

		m_Encoder_Abs[0] = - (encoder_value - 512) * 360 / (1024*3.3);// 303 = Offset // 512 = Centro
	//Calibracion inicial de la posicion del encoder relativo.
	if (red)
	{
		ang_inicial = m_Encoder_Abs[0];
		aux = m_enc_inc;
		red = false;
	}
	m_Encoder[0] = m_enc_inc - aux + ang_inicial;
	// ROS_INFO("Encoder Inc: %f ", m_enc_inc);
	ROS_INFO_COND_NAMED( m_Encoder[0] !=  m_Encoder[1], " test only " , "Encoder: %f ", m_Encoder[0]);
	// ROS_INFO("Encoder Abs: %f ", m_Encoder_Abs[0]);

	// Proteccion que avisa de la discrepancia de datos entre encoders y recalibra el incremental
	if (std::abs(m_Encoder[0]-m_Encoder_Abs[0])>5)
	{
		red = true;
		ROS_WARN("La diferencia entre encoders es mayor de 2 grados. Se produce recalibracion");
	}

	/*Lectura del encoder de la direccion y predictor de smith de la velocidad*/
	rpm = (m_Encoder[0] - m_Encoder[1]) / 0.05;
	// m_ys[0] = m_ys[1] * 0.1709 - 0.0775 * m_us[1+3]; //Versión 17/7/18
	m_ys[0] = m_ys[1] * 0.1709 - 0.0939 * m_us[1+3];// Versión 17/9/25

	// If para la division de los modos de control
	// --------------------------------------------

	// Modo manual
	if (Manual_Control)
	{
			ROS_INFO_ONCE("Controlador eCAR en modo manual"); /*Sustituir por ROS_THROTTLE_INFO???*/

		// PWM
		m_us[0] = round(Eje_x * 254);
		/*	Protección que detecta que el encoder está en el límite y solo permite girar en el sentido contrario*/
		if (std::abs(m_Encoder[0]) >= max_p)
			lim = 1;
		if (std::abs(m_Encoder[0]) <= (max_p - 5) && lim == 1)
			lim = 0;
		if (lim == 1)
		{
			ROS_WARN("El mecanismo se encuentra proximo al extremo");
			if(m_Encoder[0] > 0 && m_us[0] > 0)
				m_us[0] = 0;
			if(m_Encoder[0] < 0 && m_us[0] < 0)
				m_us[0] = 0;
		}

		if (m_us[0] < 0)
			msg_b.data = false;
		else
			msg_b.data = true;

		msg_ui.data = abs(m_us[0]);
		m_pub_rev_steering.publish(msg_b);
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO_COND_NAMED( m_us[0] !=  m_us[1], " test only " , "PWM: %i ", m_us[0]);
	}

	// Modo automatico
	else
	{
		ROS_INFO_ONCE("Controlador eCAR en modo automatico");

	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ */

	/*	Lectura de la referencia de posicion */
		m_R_steer[0]	= (double)(Eje_x * 50);		//Implemenctación para la referencia sin filtro
	//	m_R_steer_f[0]	= (double)(Eje_x * 50);		// Implementación para la referencia con filtro
	/*	Filtro en la referencia para disminuir la sobreoscilación en la señal de salida */
	//	m_R_steer[0] = 0.9649 * m_R_steer[1] + 0.0351 * m_R_steer_f[0];

	/*	Saturación de la referencia para protección contra sobrecorrientes*/ //Versión 17/7/18
		double sat_ref = 4.55;
		double pendiente = (m_R_steer[0] - m_R_steer[1]) / 0.05;
		if (pendiente >= sat_ref)
			m_R_steer[0] = (m_R_steer[1] + sat_ref);

	/*	Corrección del sentido de las ruedas*/
		m_R_steer[0] = - m_R_steer[0];

		ROS_INFO("Referencia: %f ", m_R_steer[0]);

	/*	Calculo del error al restar el encoder a la referencia de posicion */
		m_ep[0] = m_R_steer[0] - m_Encoder[0];

	/*	Controlador lazo externo */
		m_up[0] = m_up[1] + 2.9082 * m_ep[0] - 1.8240 * m_ep[1]; //Versión 17/7/18
		//m_up[0] = m_up[1] + 11.1420 * m_ep[0] - 19.9691 * m_ep[1] - 8.8889 * m_ep[2];  // Versión 17/9/25

	/*	Calcular el error del segundo lazo restando el valor de la velocidad determinada en la iteracion anterior */
		m_es[0] = m_up[0] - m_ys[0]; // - (rpm - m_ys[3]);

	/*	Introduccion de la ecuacion del controlador */
		m_us[0] = round(m_us[1] - 2.8261 * m_es[0] - 0.1750 * m_es[1]);//Versión 17/7/18
		//m_us[0] = round(m_us[1] - 2.3522 * m_es[0] - 0.1420 * m_es[1]);// Versión 17/9/25

		int m_v= m_us[0]; // Variable para el mecanismo antiwindup // Versión 17/9/25

	/*	Protección que detecta que el encoder esta en el limite y solo permite girar en el sentido contrario*/
		if (std::abs(m_Encoder[0]) >= max_p)
		lim = 1;
		if (std::abs(m_Encoder[0]) <= (max_p - 5) && lim == 1)
		lim = 0;
		if (lim ==1)
		{
			ROS_WARN("El mecanismo se encuentra próximo al extremo, reduzca la referencia");
			if(m_Encoder[0] > 0 && m_us[0] > 0)
				m_us[0] = 0;
			if(m_Encoder[0] < 0 && m_us[0] < 0)
				m_us[0] = 0;
		}
		ROS_INFO("Señal de control: %i ", m_us[0]);
	/*	Implementar saturacion */
		if (m_us[0] > 254)
		{
			ROS_WARN("Saturacion superior");
			m_us[0] = 254;
		}
		if (m_us[0] < -254)
		{
			ROS_WARN("Saturacion inferior");
			m_us[0] = -254;
		}
	/*	Mecanismo Anti-windup para proteccion*/
		if(m_us[0] - m_v != 0)
		{
			ROS_WARN("Activacion del mecanismo anti-windup");
			m_antiwindup[0] = (m_us[0] - m_v) / sqrt(0.0283);
		}
		else
			m_antiwindup[0] = 0;

		m_u[0] = round(0.5 * (2 * m_u[1] + 0.05 * (m_antiwindup[0] + m_antiwindup[1])));
		ROS_INFO("Señal de control + Antiwindup: %i ", m_u[0]);
		ROS_INFO("Yp: %f, Encoder: %f, Ep: %f, Up: %f, Ys: %f, Es: %f, Us: %i", m_yp[0],rpm, m_ep[0], m_up[0],m_ys[0],m_es[0],m_us[0]);

	/*	Envio de datos a los parametros correspondientes de ROS*/
		if (m_us[0] < 0)
			msg_b.data = false;
		else
			msg_b.data = true;

		msg_ui.data = abs(m_us[0]);
		m_pub_rev_steering.publish(msg_b);
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", msg_ui.data);

	}
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
		voltaje_pedal = 1.0 + std::abs(Eje_y) * 4.76;
/*
		if (Eje_y<0)
		{
			GPIO7 = true;
		}
		else
		{
			GPIO7 = false;
		}
*/
		msg_f.data = voltaje_pedal;
		m_pub_voltage_pedal.publish(msg_f);
/*
		ROS_INFO("Pedal: %.02f volts", voltaje_pedal);
		// Bool
		
		msg_b.data = GPIO7;
		m_pub_rev_relay.publish(msg_b);

		ROS_INFO("Rev Relay = %s", GPIO7 ? "true":"false");

	/* Actualizacion de valores*/
	m_R_steer[1] = m_R_steer[0];
	m_R_steer_f[1] = m_R_steer_f[0];
	m_u[1] = m_u[0];
	m_antiwindup[1] = m_antiwindup[0];
	for (int i=2;i>=1;i--)
	{
		m_yp[i] = m_yp[i-1];
		m_ep[i] = m_ep[i-1];
	}
	for (int i=5;i>=1;i--)
	{
		m_up[i] = m_up[i-1];
	}
	m_es[1] = m_es[0];
	m_Encoder[1] = m_Encoder[0];
	for (int i=3;i>=1;i--)
	{
		m_ys[i] = m_ys[i-1];
	}

	for (int i=4;i>=1;i--)
	{
		m_us[i] = m_us[i-1];
	}
	return true;
}


void CSteerControllerLowLevel::statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("Status Mode: %s", msg->data ? "true":"false" );
	Status_mode = msg->data;
}

void CSteerControllerLowLevel::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	//ROS_INFO("Steer_axis %.02f", msg->data );
	Eje_x = msg->data;
}

void CSteerControllerLowLevel::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	//ROS_INFO("Voltage pedal %.02f", msg->data );
	Eje_y = msg->data;
}

void CSteerControllerLowLevel::GPIO7Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//ROS_INFO("Reverse throttle direction : %s", msg->data ? "true":"false" );
	GPIO7 = msg->data;
}

void CSteerControllerLowLevel::encoderCallback(const arduino_daq::EncodersReading::ConstPtr& msg)
{
	m_enc_inc = - (msg->encoder_values[0]) / ((500 * 100 * 3.3)/109.0909); //(ppv * reductor * nºvueltas)/ang_max ==(500ppv * 100:1 * 3.3v)/109.0909º
//	m_Encoder_m[0] = (msg->encoder_values[1]);	// Comprobar valor del encoder
}
void CSteerControllerLowLevel::encoderAbsCallback(const arduino_daq::EncoderAbsReading::ConstPtr& msg)
{
	m_Encoder_Absoluto = (msg->encoder_value);

}