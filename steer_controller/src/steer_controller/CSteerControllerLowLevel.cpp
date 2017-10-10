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
float Tm = 0.05;			/*Tiempo entre iteraciones en segundos*/
bool Status_mode;			/*Variable para la comprobacion del modo de control*/
bool GPIO7 = false;			/*Variable asosciada al rele de la marcha del vehículo*/
int dep = 0;				/*Variable auxiliar para mostrar mensaje del modo de control en consola*/
double m_Encoder_Absoluto;	/*Variable para la lectura del encoder absoluto*/
int lim = 0;				/*Variable auxiliar indicadora si el mecanismo se encuentra proximo al extremo y evitar que continue avanzando*/
double aux = 0;				/*Variable auxiliar indicadora de la posicion del encoder incremental en el momento de la recalibracion*/
double ang_inicial = 0;		/*Variable auxiliar indicadora de la posicion del encoder absoluto en el momento de la recalibracion*/
double m_enc_inc = 0;		/*Valor actual del encoder incremental*/
bool red = true;			/*Variable auxiliar indicadora si es necesaria la recalibracion*/

double pos_ant = 0;			/*Valor de la posicion en la iteracion anterior del encoder absoluto*/
bool paso = false;			/*Variable auxiliar indicadora de si el encoder absoluto ha pasado del valor 1024*/

/*CONTROLADOR*/
// Predictor
float a1_p = - 0.1709;
float b1_p = - 0.0775;

/*PROTECCIONES*/
	int max_p = 50;			/*Valor maximo que puede alcanzar la direccion sin sufrir daños*/
	double sat_ref = 4.55;	/*Valor maximo que puede variar la referencia de posicion entre iteraciones*/
	int sat_abs = 254;		/*Valor máximo que puede alcanzar la señal de control introducida en el sistema*/

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
		b.data = true;
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
	bool b2,b1,b3;

	//Pendiente de crear el nodo correpondiente que comunique estos parametros
		m_q_ext[0] = 1.8903;
		m_q_ext[1] = - 1.8240;
		m_q_ext[2] = 0;
		m_q_int[0] = - 2.85;
		m_q_int[1] = - 0.1765;
		m_q_int[2] = 0;
		m_us[0] = (int)(m_us[1] + m_q_int[0] * m_es[0] - 0.1765 * m_es[1]);

	// Lectura del modo de control
	bool Manual_control = Status_mode;

	std_msgs::UInt8 msg_ui;
	std_msgs::Float64 msg_f;
	std_msgs::Bool msg_b;

	//Calibracion encoder Absoluto
	if(pos_ant == 0 && m_Encoder_Absoluto < 280)					/*Comprobacion por si el encoder se encuentra en una posicion superior a 1024*/
		paso = true;

	if(m_Encoder_Absoluto - pos_ant < - 500 || paso == true)		/*Comprobacion por si el encoder se inicia en una posicion superior a 1024 o*/
	{																/* se produce un salto durante la operacion desde 1024 a 0*/
		encoder_value = m_Encoder_Absoluto - 303 + 1024;			/*Correccion del valor del encoder y del offset*/
		paso = true;												/*Variable que indica que el encoder opera en posiciones superiores a 1024*/
		if(m_Encoder_Absoluto + 1024 > 1500)
		paso = false;
	}
	if(paso == false)
		encoder_value = m_Encoder_Absoluto - 303;

	pos_ant = m_Encoder_Absoluto;
	// 150.6995 se corresponde a la correlación entre un paso del encoder absoluto y el encoder incremental
	// 1824.9 se corresponde con (ppv * reductor * nºvueltas)/ang_max
	m_Encoder_Abs[0] = - (encoder_value - 512) * 150.6995 / 1824.9;// 303 = Offset // 512 = Centro
	ROS_INFO_COND_NAMED( m_Encoder_Abs[0] !=  m_Encoder_Abs[1], " test only " , "Encoder_Abs: %f ", m_Encoder_Abs[0]);

	//Calibracion inicial de la posicion del encoder relativo. Produce errores
/*	if (red)
	{
		ang_inicial = m_Encoder_Abs[0];
		aux = m_enc_inc;
		red = false;
	}
	m_Encoder[0] = m_enc_inc - aux + ang_inicial; */
	m_Encoder[0] = m_enc_inc;
	// ROS_INFO_COND_NAMED( m_Encoder[0] !=  m_Encoder[1], " test only " , "Encoder: %f ", m_Encoder[0]);

	// Proteccion que avisa de la discrepancia de datos entre encoders y recalibra el incremental
/*	if (std::abs(m_Encoder[0]-m_Encoder_Abs[0])>5)
	{
		red = true;
		ROS_WARN("La diferencia entre encoders es mayor de 2 grados. Se produce recalibracion");
	}
*/
	/*Lectura del encoder de la direccion y predictor de smith de la velocidad*/
	rpm = (m_Encoder[0] - m_Encoder[1]) / Tm;
	m_ys[0] = - m_ys[1] * a1_p + b1_p * m_us[1+3];

	// If para la division de los modos de control
	// --------------------------------------------

	// Modo manual
	if (Manual_control)
	{	
		ROS_INFO_ONCE("Controlador eCAR en modo manual");
		// PWM
		m_us[0] = round(Eje_x * sat_abs);

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

		ROS_INFO_COND_NAMED( m_us[0] !=  m_us[1] || m_Encoder[0] !=  m_Encoder[1], " test only " , "PWM: %i & Encoder: %f", msg_ui.data, m_Encoder[0]);

	}

	// Modo automatico
	else
	{	
		ROS_INFO_ONCE("Controlador eCAR en modo automatico");

	/*	+-------------------+
		|	STEER-BY-WIRE	|
		+-------------------+ 
	*/
	/*	Lectura de la referencia de posicion */
		m_R_steer[0] = (double)(- Eje_x * (max_p - 10));	/* El valor -10 es temporal hasta verificar el correcto y robusto funcionamiento del sistema

		/*	Saturación de la referencia para protección contra sobrecorrientes*/ //Versión 17/7/18
		double pendiente = (m_R_steer[0] - m_R_steer[1]) / Tm;
		if (pendiente >= sat_ref)
			m_R_steer[0] = (m_R_steer[1] + sat_ref);

		/*	Corrección del sentido de las ruedas*/
		m_R_steer[0] = - m_R_steer[0];

		ROS_INFO("Referencia: %f ", m_R_steer[0]);

	/*	Calculo del error al restar la restar el encoder de la interior iteracion a la referencia de posicion */
		m_ep[0] = m_R_steer[0] - m_Encoder[0]; //- m_yp[0] -(m_yp[0]-m_Encoder[0]); Realimentación para predictor de Smith

	/*	Controlador lazo externo */
		m_up[0] = m_up[1] + m_q_ext[0] * m_ep[0] + m_q_ext[1] * m_ep[1] + m_q_ext[2] * m_ep[2];

	/*	Mecanismo Anti-windup para protección*/

	/*	Calcular el error del segundo lazo restando el valor de la velocidad determinada en la iteracion anterior */
		m_es[0] = m_up[0] - m_ys[0];/* - (rpm - m_ys[3]);*/

	/*	Introduccion de la ecuacion del controlador */
		m_us[0] = (int)(m_us[1] + m_q_int[0] * m_es[0] + m_q_int[1] * m_es[1] + m_q_int[2] * m_es[2]);

	/*	Implementar saturacion */
		if (m_us[0] > sat_abs)
		{
			m_us[0] = sat_abs;
		}
		if (m_us[0] < -sat_abs)
		{
			m_us[0] = -sat_abs;
		}
		ROS_INFO("Yp: %f, Encoder: %f, Ep: %f, Up: %f, Ys: %f, Es: %f, Us: %i", m_yp[0],rpm, m_ep[0], m_up[0],m_ys[0],m_es[0],m_us[0]);
	
	/*	Envio de datos a los parametros correspondientes de ROS*/
		if (m_us[0] < 0)
		{
			msg_ui.data = - m_us[0];
			msg_b.data = false;
		}
		else
		{
			msg_ui.data = m_us[0];
			msg_b.data = true;
		}
		m_pub_rev_steering.publish(msg_b);
		m_pub_pwm_steering.publish(msg_ui);

		ROS_INFO("PWM: %i ", msg_ui.data);
	}
	// Este sistema se debe desarrollar en un nodo nuevo
	/*	+-----------------------+
		|	THROTTLE-BY-WIRE	|
		+-----------------------+
	*/
		voltaje_pedal = 1.0 + std::abs(Eje_y) * 4.76;

		if (voltaje_pedal < 1)
			voltaje_pedal = 1;


		msg_f.data = voltaje_pedal;
		m_pub_voltage_pedal.publish(msg_f);

		ROS_INFO("Pedal: %.02f volts", voltaje_pedal);
		// Bool
		
		b1 = GPIO7;
		msg_b.data = GPIO7;
		m_pub_rev_relay.publish(msg_b);

		ROS_INFO("Rev Relay = %s", b1 ? "true":"false");

	/* Actualizacion de valores*/
	m_R_steer[1] = m_R_steer[0];
	//m_R_steer_f[1] = m_R_steer_f[0];
	//m_u[1] = m_u[0];
	//m_antiwindup[1] = m_antiwindup[0];
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
	Status_mode = msg->data;
}

void CSteerControllerLowLevel::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	Eje_x = msg->data;
}

void CSteerControllerLowLevel::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	Eje_y = msg->data;
}

void CSteerControllerLowLevel::GPIO7Callback(const std_msgs::Bool::ConstPtr& msg)
{
	GPIO7 = msg->data;
}

void CSteerControllerLowLevel::encoderCallback(const arduino_daq::EncodersReading::ConstPtr& msg)
{
	m_enc_inc = (msg->encoder_values[0]) / 1824.9; //( ppv * reductor * n vueltas ) / ang_max 
	//	m_Encoder_m[0] = (msg->encoder_values[1]);	// Comprobar valor del encoder
}

void CSteerControllerLowLevel::encoderAbsCallback(const arduino_daq::EncoderAbsReading::ConstPtr& msg)
{
	m_Encoder_Absoluto = (msg->encoder_value);
}