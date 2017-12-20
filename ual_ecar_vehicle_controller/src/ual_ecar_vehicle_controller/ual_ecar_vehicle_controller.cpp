/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <ual_ecar_vehicle_controller/VehicleControllerLowLevel.h>
#include <thread>
#include <ros/console.h>
#include <ual_ecar_vehicle_controller/SteerControllerStatus.h>
#include <cstring>
#include <array>
#include <iostream>

// Define to see serial communication traces
#define DEBUG_TRACES

/* +------------------------+
   |		VARIABLES		|
   +------------------------+*/

bool mode_manual = false;	/*Variable para la comprobacion del modo de control*/

bool VehicleControllerLowLevel::initialize()
{
	ROS_INFO("VehicleControllerLowLevel::inicialize() ok.");

	m_serial_port_name = "/dev/serial/by-id/usb-UAL_Claraquino_#1__FT232R_USB_UART__A71VGDJN-if00-port0";
	m_serial_port_baudrate = 500000;
	m_nh_params.getParam("SERIAL_PORT",m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);

	// Try to connect...
	if (this->AttemptConnection())
	{
		ROS_INFO("Connection OK to VehicleLowLevelController [Claraquino].");
	}
	else
	{
		ROS_ERROR("Error in VehicleControllerLowLevel::AttemptConnection()!");
		return false;
	}

	m_pub_controller_status = m_nh.advertise<ual_ecar_vehicle_controller::SteerControllerStatus>("vehicle_controller_status", 10);
	/*Pub: Encoders, Control signal, ADC*/

	m_sub_contr_status	= m_nh.subscribe("vehicle_manual_mode", 10, &VehicleControllerLowLevel::statusCallback, this);
	m_sub_eje_x  		= m_nh.subscribe("joystick_eje_x", 10, &VehicleControllerLowLevel::ejexCallback, this);
	m_sub_eje_y			= m_nh.subscribe("joystick_eje_y", 10, &VehicleControllerLowLevel::ejeyCallback, this);
	/*Sub:	1. Joystick[Axis & control modes]
			2. System_Identification[Controller & Smith predictor params, Feedforwards...]
	*/
	
	return true;
}

void VehicleControllerLowLevel::processIncommingFrame(const std::vector<uint8_t> &rxFrame)
{
	//MRPT_LOG_INFO_STREAM  << "Rx frame, len=" << rxFrame.size();
	if (rxFrame.size() >= 5)
	{
		switch (rxFrame[1])
		{
			case RESP_ADC_READINGS:
			{
				TFrame_ADC_readings rx;
				::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewADCCallback(rx.payload);
			}
			break;

			case RESP_ENCODER_READINGS:
			{
				TFrame_ENCODERS_readings rx;
				::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewENCCallback(rx.payload);
			}
			break;

			case RESP_EMS22A_READINGS:
			{
				TFrame_ENCODER_ABS_reading rx;
				::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewENCAbsCallback(rx.payload);
			}
			break;
		};
	}
}


bool VehicleControllerLowLevel::iterate()
{
	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;

	if (!m_serial.isOpen())
	{
		if (!this->initialize())
			return false;
	}

	std::vector<uint8_t> rxFrame;
	while (ReceiveFrameFromController(rxFrame) && ++nFrames<MAX_FRAMES_PER_ITERATE)
	{
		// Process them:
		processIncommingFrame(rxFrame);
	}

	// if no frame was received, ping the uC to keep comms alive:
	if (!nFrames && m_NOP_sent_counter++>20)
	{
		m_NOP_sent_counter=0;

		// Send a dummy NOP command
		TFrameCMD_NOP cmd;
		cmd.calc_and_update_checksum();
		return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
	}

	return true;
}

void VehicleControllerLowLevel::statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	mode_manual = msg->data;
	MRPT_TODO("Enviar al controlador!?");

}

void VehicleControllerLowLevel::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	const double eje_x = msg->data;

	MRPT_TODO("Enviar al controlador!?");
}

void VehicleControllerLowLevel::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	const double eje_y = msg->data;
	MRPT_TODO("Enviar al controlador!?");

}

void VehicleControllerLowLevel::daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data)
{
	MRPT_TODO("Append data to ControllerStatus");
#if 0
	arduino_daq::AnalogReading msg;

	msg.timestamp_ms = data.timestamp_ms;
	for (int i=0;i<sizeof(data.adc_data)/sizeof(data.adc_data[0]);i++) {
		 msg.adc_data[i] = data.adc_data[i];
	}

	m_pub_ADC.publish(msg);
#endif
}

void VehicleControllerLowLevel::daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data)
{
	MRPT_TODO("Append data to ControllerStatus");
#if 0
	arduino_daq::EncodersReading msg;

	msg.timestamp_ms = data.timestamp_ms;
	msg.period_ms = data.period_ms;
	const int N =sizeof(data.encoders)/sizeof(data.encoders[0]);

	msg.encoder_values.resize(N);
	for (int i=0;i<N;i++) {
		 msg.encoder_values[i] = data.encoders[i];
	}

	m_pub_ENC.publish(msg);
#endif
}

void VehicleControllerLowLevel::daqOnNewENCAbsCallback(const TFrame_ENCODER_ABS_reading_payload_t &data)
{
	MRPT_TODO("Append data to ControllerStatus");
#if 0
	arduino_daq::EncoderAbsReading msg;

	msg.timestamp_ms   = data.timestamp_ms;
	msg.encoder_status = data.enc_status;
	msg.encoder_value  = data.enc_pos;

	m_pub_ENC_ABS.publish(msg);
#endif
}

bool VehicleControllerLowLevel::AttemptConnection()
{
	if (m_serial.isOpen()) return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100,0,10,0,50);

		MRPT_LOG_INFO_FMT("[VehicleControllerLowLevel::AttemptConnection] Serial port '%s' open was successful.", m_serial_port_name.c_str() );
		return true;
	}
	catch (std::exception &e)
	{
		MRPT_LOG_ERROR_FMT("[VehicleControllerLowLevel::AttemptConnection] COMMS error: %s", e.what() );
		return false;
	}
}


/** Sends a binary packet (returns false on COMMS error) */
bool VehicleControllerLowLevel::WriteBinaryFrame(const uint8_t *full_frame, const size_t full_frame_len)
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
			MRPT_LOG_DEBUG_FMT("Tx frame: %s", s.c_str());
		}
#endif

		m_serial.WriteBuffer(full_frame,full_frame_len);
		return true;
	}
	catch (std::exception &)
	{
		return false;
	}
}

bool VehicleControllerLowLevel::SendFrameAndWaitAnswer(
	const uint8_t *full_frame,
	const size_t full_frame_len,
	const int num_retries,
	const int retries_interval_ms,
	uint8_t expected_ans_opcode
	)
{
	if (expected_ans_opcode==0 && full_frame_len>2)
		expected_ans_opcode=full_frame[1]+0x70; // answer OPCODE convention

	for (int iter=0;iter<num_retries;iter++)
	{
		if (iter>0)
			std::this_thread::sleep_for(std::chrono::milliseconds(retries_interval_ms));

		// Send:
		if (!WriteBinaryFrame(full_frame,full_frame_len))
			continue;

		// Wait for answer:
		std::vector<uint8_t> rxFrame;
		if (this->ReceiveFrameFromController(rxFrame) && rxFrame.size()>4)
		{
			const auto RX_OPCODE = rxFrame[1];
			if (RX_OPCODE==expected_ans_opcode)
			{
				// We received the ACK from the uC, yay!
				MRPT_LOG_DEBUG_FMT("SendFrameAndWaitAnswer(): Rx ACK for OPCODE=0x%02X after %i retries.",full_frame_len>2 ? full_frame[1] : 0, iter);
				return true;
			}
			else
			{
				// Ensure the frame gets processed:
				processIncommingFrame(rxFrame);
			}
		}
	}
	MRPT_LOG_ERROR_FMT("SendFrameAndWaitAnswer(): Missed ACK for OPCODE=0x%02X",full_frame_len>2 ? full_frame[1] : 0);
	return false; // No answer!
}

bool VehicleControllerLowLevel::ReceiveFrameFromController(std::vector<uint8_t> &rxFrame)
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
			MRPT_LOG_ERROR_FMT("ReceiveFrameFromController(): Comms error: %s", e.what());
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

		// Reading was OK:
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
			MRPT_LOG_DEBUG_FMT("%s", s.c_str());
		}
#endif

	// All OK
	return true;
}

bool VehicleControllerLowLevel::CMD_GPIO_output(int pin, bool pinState)
{
    TFrameCMD_GPIO_output cmd;
    cmd.payload.pin_index = pin;
    cmd.payload.pin_value = pinState ? 1:0;

    cmd.calc_and_update_checksum();

	return SendFrameAndWaitAnswer(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the clutch
bool VehicleControllerLowLevel::CMD_DAC(int dac_index,double dac_value_volts)
{
    uint16_t dac_counts = 4096 * dac_value_volts / 5.0;
    mrpt::utils::saturate(dac_counts, uint16_t(0), uint16_t(4095));

    TFrameCMD_SetDAC cmd;
    cmd.payload.dac_index = dac_index;
    cmd.payload.dac_value_HI = dac_counts >> 8;
    cmd.payload.dac_value_LO = dac_counts & 0x00ff;
    cmd.payload.flag_enable_timeout = true;

    cmd.calc_and_update_checksum();

	return SendFrameAndWaitAnswer(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

bool VehicleControllerLowLevel::IsConnected() const
{
	return m_serial.isOpen();
}

