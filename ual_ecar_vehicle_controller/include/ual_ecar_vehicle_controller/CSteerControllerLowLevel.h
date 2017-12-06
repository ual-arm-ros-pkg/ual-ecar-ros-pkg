/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/UInt8.h>
#include <steer_controller2pc-structs.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x199
# include <mrpt/comms/CSerialPort.h>
using mrpt::comms::CSerialPort;
#else
# include <mrpt/hwdrivers/CSerialPort.h>
using mrpt::hwdrivers::CSerialPort;
#endif


class CSteerControllerLowLevel : public mrpt::utils::COutputLogger
{
public:
	CSteerControllerLowLevel() { }

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh {};
	ros::NodeHandle m_nh_params {"~"};

	ros::Publisher  m_pub_controller_status;
	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_contr_status;

	bool initialize();

	/** called when work is to be done */
	bool iterate();

  protected:
	void statusCallback(const std_msgs::Bool::ConstPtr& msg);
	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
	void daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data);
	void daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data);
	void daqOnNewENCAbsCallback(const TFrame_ENCODER_ABS_reading_payload_t &data);

	bool CMD_GPIO_output(int pin, bool pinState);
	bool CMD_DAC(int dac_index, double dac_value_volts);
	bool CMD_PWM(int pin_index, uint8_t pwm_value);

	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	CSerialPort m_serial;  //!< The serial COMMS object
	int         m_NOP_sent_counter {0};

	// Local methods:
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	void processIncommingFrame(const std::vector<uint8_t> &rxFrame);
	bool WriteBinaryFrame( const uint8_t *full_frame, const size_t full_frame_len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)
	bool SendFrameAndWaitAnswer(
		const uint8_t *full_frame,
		const size_t full_frame_len,
		const int num_retries = 10,
		const int retries_interval_ms = 40,
		uint8_t expected_ans_opcode = 0 //<! 0 means the default convention: full_frame[1]+0x70,
		); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)

};
