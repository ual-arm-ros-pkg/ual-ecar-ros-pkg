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

#include <mrpt/hwdrivers/CSerialPort.h>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <battery_charge/BatReading.h>

#include <mrpt/utils/COutputLogger.h>
#include <batterycharge2pc-structs.h>
#include <functional>


class BatteryCharge_LowLevel : public mrpt::utils::COutputLogger
{
public:
	BatteryCharge_LowLevel();
	virtual ~BatteryCharge_LowLevel();

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_params;

	std::vector<ros::Subscriber> m_sub_OPTO_outputs;
	ros::Publisher  m_pub_battery_charge;


	void setSerialPort(const std::string &sSerialName) {
		m_serial_port_name = sSerialName;
	}
	std::string getSerialPort() const {
		return m_serial_port_name;
	}

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

	bool CMD_OPTO_output(int opto, bool optoState);
	bool CMD_BAT_START(const TFrameCMD_BATTERY_start_payload_t &enc_config);
	bool CMD_BAT_STOP();

	void set_BAT_readings_callback(const std::function<void(TFrame_BATTERY_readings_payload_t)> &f) {
		m_bat_callback = f;
	}

protected:
	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	mrpt::hwdrivers::CSerialPort m_serial;  //!< The serial COMMS object

	// Local methods:
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	bool WriteBinaryFrame( const uint8_t *full_frame, const size_t full_frame_len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)

	std::function<void(TFrame_BATTERY_readings_payload_t)> m_bat_callback;
	void daqSetDigitalPinCallback(int index, const std_msgs::Bool::ConstPtr& msg);

	void daqOnNewBATCallback(const TFrame_BATTERY_readings_payload_t &data);

};
