/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-18, Universidad de Almeria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

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
struct TFrame_BATTERY_readings_payload_t;
struct TFrameCMD_VERBOSITY_CONTROL_payload_t;

class BatteryCharge_LowLevel : public COutputLogger
{
public:
	BatteryCharge_LowLevel() {}


	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_params{"~"};

	//std::vector<ros::Subscriber> m_sub_OPTO_outputs;
	ros::Publisher  m_pub_battery_voltaje, m_pub_ammeter_value;
	std::vector<ros::Subscriber> m_sub_optocoupler;

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

protected:
	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	CSerialPort m_serial;  //!< The serial COMMS object
	int m_NOP_sent_counter{0};

	std::function<void(TFrame_BATTERY_readings_payload_t)> m_bat_callback;
	void daqOnNewBATCallback(const TFrame_BATTERY_readings_payload_t& data);
	void daqOnNewADCCallback(const TFrame_ADC_readings_payload_t& data);
	void daqSetOptocouplerCallback(int pin, const std_msgs::Bool::ConstPtr& msg);
	bool CMD_Decimation_configuration(const TFrameCMD_VERBOSITY_CONTROL_payload_t& Decimation_config);
	bool CMD_Optocoupler(int pin, bool pinState);

	uint8_t m_optocoupler{0};

	// Local methods:
	void processIncommingFrame(const std::vector<uint8_t>& rxFrame);
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	bool WriteBinaryFrame( const uint8_t *full_frame, const size_t full_frame_len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)
	bool SendFrameAndWaitAnswer(const uint8_t* full_frame, const size_t full_frame_len,
		const int num_retries = 10,
		const int retries_interval_ms = 40,
		uint8_t expected_ans_opcode = 0  //<! 0 means the default convention: full_frame[1]+0x70,
	);  //!< Sends a binary packet, in the expected format  (returns false on
	//!< COMMS error)

};
