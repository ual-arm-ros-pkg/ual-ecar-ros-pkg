/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-17, Universidad de Almeria
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


#include <battery_charge/BatteryCharge_LowLevel.h>
#include <cstring>
#include <functional>
#include <array>
#include <thread>
#include <chrono>
#include <battery_charge/BatReading.h>
<<<<<<< HEAD
#include <batterycharge2pc-structs.h>
=======
>>>>>>> parent of 151beae... Nodo ROS, prueba

#include <ros/console.h>

#include <mrpt/version.h>
#if MRPT_VERSION<0x199
#include <mrpt/utils/utils_defs.h>
using mrpt::utils::VerbosityLevel;
using namespace mrpt::utils;
#else
#include <mrpt/core/format.h>
#include <mrpt/core/exceptions.h>
using mrpt::system::VerbosityLevel;
using namespace mrpt::system;
#endif


#include <iostream>

using namespace std;
using namespace mrpt;

#define DEBUG_TRACES

#if MRPT_VERSION<0x199
void log_callback(const std::string &msg, const VerbosityLevel level, const std::string &loggerName, const mrpt::system::TTimeStamp timestamp, void *userParam)
#else
void log_callback(const std::string &msg, const VerbosityLevel level, const std::string &loggerName, const mrpt::system::TTimeStamp timestamp)
#endif
{
	ROS_INFO("%s",msg.c_str());
}

BatteryCharge_LowLevel::BatteryCharge_LowLevel() :
	mrpt::utils::COutputLogger("BatteryCharge_LowLevel"),
	m_nh_params("~"),
	m_serial_port_name("/dev/serial/by-id/usb-Ual-ARM-eCAMR_Monitor_de_baterías_AL1L20Ez-if00-port0"), //Cambiar
	m_serial_port_baudrate(115200)
{
#if MRPT_VERSION<0x199
	this->logRegisterCallback(&log_callback, this);
#else
	this->logRegisterCallback(&log_callback);
#endif

#ifdef DEBUG_TRACES
	this->setMinLoggingLevel(LVL_DEBUG);
#endif
}

BatteryCharge_LowLevel::~BatteryCharge_LowLevel()
{
}

bool BatteryCharge_LowLevel::initialize()
{
<<<<<<< HEAD
	ROS_INFO("BatteryCharge_LowLevel::inicialize() ok.");
	m_serial_port_name ="/dev/serial/by-id/usb-Ual-ARM-eCARM_Monitor_de_baterÃ­as_AL1L20EZ-if00-port0";
	m_serial_port_baudrate = 500000;
=======
>>>>>>> parent of 151beae... Nodo ROS, prueba
	m_nh_params.getParam("SERIAL_PORT",m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);

	// Try to connect...
	if (this->AttemptConnection())
	{
<<<<<<< HEAD
		ROS_INFO("Connection OK to BatteryCharge Monitor.");
=======
		MRPT_LOG_INFO("Connection OK to BatteryCharge Monitor.");
>>>>>>> parent of 151beae... Nodo ROS, prueba
	}
	else
	{
		MRPT_LOG_ERROR("Error in BatteryCharge::AttemptConnection()!");
		return false;
	}

	// Subscribers: OPTO outputs
	m_sub_OPTO_outputs.resize(6);
	for (int i=0;i<6;i++) {
		auto fn = boost::bind(&BatteryCharge_LowLevel::daqSetDigitalPinCallback, this, i, _1);
		m_sub_OPTO_outputs[i] = m_nh.subscribe<std_msgs::Bool>( mrpt::format("m_sub_OPTO_outputs%i",i), 10, fn);
	}

	// Publisher: battery_charge data
	m_pub_battery_charge = m_nh.advertise<battery_charge::BatReading>("m_pub_battery_charge", 10);

<<<<<<< HEAD
// Decimation params
	{
		TFrameCMD_VERBOSITY_CONTROL_payload_t Decimation_config;
		int decimate_BAT = 10,	decimate_CPU = 10000;

		m_nh_params.getParam("DECIM_BAT", decimate_BAT);
		m_nh_params.getParam("DECIM_CPU", decimate_CPU);

		if (decimate_BAT > 0 && decimate_CPU > 0)
		{
			Decimation_config.decimate_BAT = decimate_BAT;
			Decimation_config.decimate_CPU = decimate_CPU;

			MRPT_LOG_INFO_FMT(" Firmware Decimation: BAT=%i CPU=%i",decimate_BAT, decimate_CPU);
			this->CMD_Decimation_configuration(Decimation_config);
		}
	}

	return true;
}

void BatteryCharge_LowLevel::processIncommingFrame(const std::vector<uint8_t>& rxFrame)
{
	if (rxFrame.size() >= 5)
	{
		switch (rxFrame[1])
		{
			case RESP_BAT_READINGS:
			{
				TFrame_BATTERY_readings rx;
				::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewBATCallback(rx.payload);
			}
			break;
		};
	}
}

=======

return true;
}

>>>>>>> parent of 151beae... Nodo ROS, prueba
bool BatteryCharge_LowLevel::iterate()
{
	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;

	if (!m_serial.isOpen())
		return false;

	std::vector<uint8_t> rxFrame;
	while (ReceiveFrameFromController(rxFrame) && ++nFrames<MAX_FRAMES_PER_ITERATE)
	{
		// Process them:
		//MRPT_LOG_INFO_STREAM  << "Rx frame, len=" << rxFrame.size();
		if (rxFrame.size() >= 5)
		{
			TFrame_BATTERY_readings rx;
			::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));

			if (m_bat_callback)
				m_bat_callback(rx.payload);

			daqOnNewBATCallback(rx.payload);

		}

	}

	// if no frame was received, ping the uC to keep comms alive:
	if (!nFrames)
	{
		// Send a dummy NOP command
		TFrameCMD_NOP cmd;
		cmd.calc_and_update_checksum();
		return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
	}

	return true;
}

void BatteryCharge_LowLevel::daqSetDigitalPinCallback(int pin, const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("OPTO: output[%i]=%s", pin, msg->data ? "true":"false" );

    if (!CMD_OPTO_output(pin,msg->data)) {
        ROS_ERROR("*** Error sending CMD_OPTO_output!!! ***");
    }
}

void BatteryCharge_LowLevel::daqOnNewBATCallback(const TFrame_BATTERY_readings_payload_t &data)
{
	battery_charge::BatReading msg;

	msg.timestamp_ms = data.timestamp_ms;

<<<<<<< HEAD

	const double K_adc = 5.0/(1<<16);
	const double K_r = 1.0/64.4;

	msg.bat_volts.resize(N);
	for (int i=0;i<N;i++)
	{
		msg.bat_volts[i] = data.bat_volts[i]*K_adc*K_r;
	}
	msg.bat_current = data.bat_current;


=======
	for (int i=0;i<sizeof(data.bat_volts)/sizeof(data.bat_volts[0]);i++) {
		msg.bat_volts[i] = data.bat_volts[i];
	}
	
>>>>>>> parent of 151beae... Nodo ROS, prueba
	m_pub_battery_charge.publish(msg);
}


bool BatteryCharge_LowLevel::AttemptConnection()
{
	if (m_serial.isOpen()) return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100,0,10,0,50);

		return true;
	}
	catch (std::exception &e)
	{
		MRPT_LOG_ERROR_FMT("[BatteryCharge_LowLevel::AttemptConnection] COMMS error: %s", e.what() );
		return false;
	}
}


/** Sends a binary packet (returns false on COMMS error) */
bool BatteryCharge_LowLevel::WriteBinaryFrame(const uint8_t *full_frame, const size_t full_frame_len)
{
	if (!AttemptConnection()) return false;

	ASSERT_(full_frame!=NULL);

	try
	{
#ifdef DEBUG_TRACES
		{
			std::string s;
			s+=mrpt::format("TX frame (%u bytes): ", (unsigned int) full_frame_len);
			for (size_t i=0;i< full_frame_len;i++)
				s+=mrpt::format("%02X ", full_frame[i]);
			MRPT_LOG_INFO_FMT("Tx frame: %s", s.c_str());
		}
#endif

#if MRPT_VERSION<0x199
		m_serial.WriteBuffer(full_frame,full_frame_len);
#else
		m_serial.Write(full_frame,full_frame_len);
#endif
		return true;
	}
	catch (std::exception &)
	{
		return false;
	}
}

bool BatteryCharge_LowLevel::ReceiveFrameFromController(std::vector<uint8_t> &rxFrame)
{
	rxFrame.clear();
	size_t	nFrameBytes = 0;
	std::vector<uint8_t> buf;
	buf.resize(0x10000);
	buf[0] = buf[1] = 0;

	size_t	lengthField;

	/*
	START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
	  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96
	*/

	//                                   START_FLAG     OPCODE + LEN       DATA      CHECKSUM +  END_FLAG
	while ( nFrameBytes < (lengthField=(    1        +     1   +  1    +  buf[2]  +     1     +     1      )  ) )
	{
		if (lengthField>200)
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[1]=buf[2]=0;
			MRPT_LOG_INFO("[rx] Reset frame (invalid len)");
		}

		size_t nBytesToRead;
		if (nFrameBytes<3)
			nBytesToRead = 1;
		else
			nBytesToRead = (lengthField) - nFrameBytes;

		size_t 	nRead;
		try
		{
			nRead = m_serial.Read( &buf[0] + nFrameBytes, nBytesToRead );
		}
		catch (std::exception &e)
		{
			// Disconnected?
			std::cerr << "[BatteryCharge_LowLevel::ReceiveFrameFromController] Comms error: " << e.what() << std::endl;
			return false;
		}

		if ( !nRead && !nFrameBytes )
		{
			//cout << "[rx] No frame (buffer empty)\n";
			return false;
		}

		if (nRead<nBytesToRead)
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1ms);
		}

		// Lectura OK:
		// Check start flag:
		bool is_ok = true;

		if (!nFrameBytes && buf[0]!= FRAME_START_FLAG )
		{
			is_ok = false;
			MRPT_LOG_DEBUG("[rx] Reset frame (start flag)");
		}

		if (nFrameBytes>2 && nFrameBytes+nRead==lengthField)
		{
			if (buf[nFrameBytes+nRead-1]!=FRAME_END_FLAG)
			{
				is_ok= false;
				MRPT_LOG_DEBUG("[rx] Reset frame (end flag)");
			}
			//else { cout << "[rx] Frame OK\n"; }
		}

		MRPT_TODO("Checksum");

		if (is_ok)
		{
			nFrameBytes+=nRead;
		}
		else
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[1]=buf[2]=0;
		}
	}

	// Frame received
	lengthField= buf[2]+5;
	rxFrame.resize(lengthField);
	::memcpy( &rxFrame[0], &buf[0], lengthField);

#ifdef DEBUG_TRACES
		{
			std::string s;
			s+=mrpt::format("RX frame (%u bytes): ", (unsigned int) lengthField);
			for (size_t i=0;i< lengthField;i++)
				s+=mrpt::format("%02X ", rxFrame[i]);
			MRPT_LOG_INFO_FMT("%s", s.c_str());
		}
#endif

	// All OK
	return true;
}


bool BatteryCharge_LowLevel::CMD_OPTO_output(int pin_index, bool opto_value)
{
	TFrameCMD_OPTO_output cmd;
	cmd.payload.pin_index = pin_index;
	cmd.payload.pin_value = opto_value;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

bool BatteryCharge_LowLevel::IsConnected() const
{
	return m_serial.isOpen();
}

bool BatteryCharge_LowLevel::CMD_BAT_START(const TFrameCMD_BATTERY_start_payload_t &bat_config)
{
	TFrameCMD_BATTERY_start cmd;
	cmd.payload = bat_config;
	cmd.calc_and_update_checksum();

<<<<<<< HEAD
	return SendFrameAndWaitAnswer(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
=======
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}
bool BatteryCharge_LowLevel::CMD_BAT_STOP()
{
	TFrameCMD_BATTERY_stop cmd;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
>>>>>>> parent of 151beae... Nodo ROS, prueba
}
