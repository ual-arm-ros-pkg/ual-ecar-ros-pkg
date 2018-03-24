/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/UInt8.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/system/COutputLogger.h>
using mrpt::comms::CSerialPort;
using mrpt::system::COutputLogger;
#else
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/COutputLogger.h>
using mrpt::hwdrivers::CSerialPort;
using mrpt::utils::COutputLogger;
#endif

// Fwrd:
struct TFrame_ADC_readings_payload_t;
struct TFrame_ENCODERS_readings_payload_t;
struct TFrame_ENCODER_ABS_reading_payload_t;
struct TFrame_CONTROL_SIGNAL_payload_t;
struct TFrameCMD_VERBOSITY_CONTROL_payload_t;
struct TFrame_STEER_CONTROL_SIGNAL_payload_t;
struct TFrame_SPEEDCRUISE_CONTROL_SIGNAL_payload_t;


class VehicleControllerLowLevel : public COutputLogger {
public:
  VehicleControllerLowLevel() {}

/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the
* last NodeHandle destructed will close down the node.
*/
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_controller_status, m_pub_Steer_ADC, m_pub_SpeedCruise_ADC, m_pub_Steer_ENC, m_pub_SpeedCruise_ENC, m_pub_ENC_ABS, m_pub_Steer_Control_signal, m_pub_SpeedCruise_Control_signal;

	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_contr_status[2], m_sub_autonomous_driving;

	/*Sub:	1. System_Identification[Controller & Smith predictor params,
	* Feedforwards...]
	*/
bool initialize();

/** called when work is to be done */
bool iterate();


protected:
void modeSteeringCallback(const std_msgs::Bool::ConstPtr &msg);
void modeThrottleCallback(const std_msgs::Bool::ConstPtr &msg);
void autonomousModeCallback(const std_msgs::Bool::ConstPtr &msg);
void ejexCallback(const std_msgs::Float64::ConstPtr &msg);
void ejeyCallback(const std_msgs::Float64::ConstPtr &msg);
void daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data, CSerialPort &m_serial);
void daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data, CSerialPort &m_serial);
void daqOnNewENCAbsCallback(const TFrame_ENCODER_ABS_reading_payload_t &data);
void daqOnNewSteerControlSignalCallback(const TFrame_STEER_CONTROL_SIGNAL_payload_t &data);
void daqOnNewSpeedCruiseControlSignalCallback(const TFrame_SPEEDCRUISE_CONTROL_SIGNAL_payload_t &data);

bool CMD_Decimation_configuration(const TFrameCMD_VERBOSITY_CONTROL_payload_t &Decimation_config, CSerialPort &m_serial);


std::string m_serial_Steer_port_name, m_serial_SpeedCruise_port_name;
int m_serial_Steer_port_baudrate, m_serial_SpeedCruise_port_baudrate;
CSerialPort m_serial_Steer, m_serial_SpeedCruise; //!< The serial COMMS object
int m_NOP_sent_counter{0};

bool m_mode_openloop_steer{true}; /*Variable para la comprobacion del modo de control*/
bool m_mode_openloop_throttle{true}; /*Variable para la comprobacion del modo de control*/
bool m_mode_steer_changed{true};
bool m_mode_throttle_changed{true};
bool m_modes_changed{true};
bool m_autonomous_driving_mode{false};
bool m_joy_changed{false};
double m_joy_x{.0}, m_joy_y{.0};

// Local methods:
bool AttemptConnection(CSerialPort &m_serial,std::string &m_serial_port_name, int &m_serial_port_baudrate); //!< Returns true if connected OK, false on error.
bool IsConnected() const; //!< Returns true if the serial comms are working
bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data, CSerialPort &m_serial); //!< Tries to get a framed chunk of data from the controller.
void processIncommingFrame(const std::vector<uint8_t> &rxFrame, CSerialPort &m_serial);
bool WriteBinaryFrame(const uint8_t *full_frame,const size_t full_frame_len, CSerialPort &m_serial, std::string &m_serial_port_name, int &m_serial_port_baudrate); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)
bool SendFrameAndWaitAnswer(const uint8_t *full_frame, const size_t full_frame_len,
	const int num_retries = 10, const int retries_interval_ms = 40,
	uint8_t expected_ans_opcode = 0 //<! 0 means the default convention: full_frame[1]+0x70,
	, CSerialPort &m_serial); //!< Sends a binary packet, in the expected format  (returns false on COMMS
     //!< error)
};
