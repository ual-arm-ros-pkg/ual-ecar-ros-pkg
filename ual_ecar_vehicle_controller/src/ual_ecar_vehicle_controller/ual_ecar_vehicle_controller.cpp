/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-17  University of Almeria                            |
   +---------------------------------------------------------------------------+
 */

#include <array>
#include <cstring>
#include <iostream>
#include <ros/console.h>
#include <thread>
#include <ual_ecar_vehicle_controller/AnalogReading.h>
#include <ual_ecar_vehicle_controller/ControlSignal.h>
#include <ual_ecar_vehicle_controller/EncoderAbsReading.h>
#include <ual_ecar_vehicle_controller/EncodersReading.h>
#include <ual_ecar_vehicle_controller/SteerControllerStatus.h>
#include <ual_ecar_vehicle_controller/VehicleControllerLowLevel.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
using mrpt::saturate;
#else
using mrpt::utils::saturate;
#endif

// Define to see serial communication traces
//#define DEBUG_TRACES

/* +------------------------+
   |		VARIABLES		|
   +------------------------+*/

bool VehicleControllerLowLevel::initialize() {
	ROS_INFO("VehicleControllerLowLevel::inicialize() ok.");

	m_serial_port_name = "/dev/serial/by-id/usb-UAL_Claraquino_#1__FT232R_USB_UART__A71VGDJN-if00-port0";
	m_serial_port_baudrate = 500000;
	m_nh_params.getParam("SERIAL_PORT", m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE", m_serial_port_baudrate);

	// Try to connect...
	if (this->AttemptConnection()) {
		ROS_INFO("Connection OK to VehicleLowLevelController [Claraquino].");
	} else {
		ROS_ERROR("Error in VehicleControllerLowLevel::AttemptConnection()!");
		return false;
	}

	// Publisher: Controller Status
	m_pub_controller_status = m_nh.advertise<ual_ecar_vehicle_controller::SteerControllerStatus>("vehicle_controller_status", 10);

	// Publisher: ADC data
	m_pub_ADC = m_nh.advertise<ual_ecar_vehicle_controller::AnalogReading>("claraquino_adc", 10);

	// Publisher: ENC data
	m_pub_ENC = m_nh.advertise<ual_ecar_vehicle_controller::EncodersReading>("claraquino_encoders", 10);

	// Publisher: ABS ENC data
	m_pub_ENC_ABS = m_nh.advertise<ual_ecar_vehicle_controller::EncoderAbsReading>("claraquino_abs_encoder", 10);

  // Publisher: Control signal data
	m_pub_Control_signal = m_nh.advertise<ual_ecar_vehicle_controller::ControlSignal>("claraquino_control_signal", 10);

	m_sub_contr_status[0] = m_nh.subscribe("vehicle_openloop_mode_steering", 10, &VehicleControllerLowLevel::modeSteeringCallback, this);
	m_sub_contr_status[1] = m_nh.subscribe("vehicle_openloop_mode_throttle", 10, &VehicleControllerLowLevel::modeThrottleCallback, this);
	m_sub_eje_x = m_nh.subscribe("joystick_eje_x", 10, &VehicleControllerLowLevel::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe("joystick_eje_y", 10, &VehicleControllerLowLevel::ejeyCallback, this);
	m_sub_autonomous_driving =m_nh.subscribe("vehicle_autonomous_mode", 10, &VehicleControllerLowLevel::autonomousModeCallback, this);
	/*Sub:	1. Joystick[Axis & control modes]
                  2. System_Identification[Controller & Smith predictor params,
		Feedforwards...]
	*/

	return true;
}

void VehicleControllerLowLevel::processIncommingFrame(const std::vector<uint8_t> &rxFrame) {
	// MRPT_LOG_INFO_STREAM  << "Rx frame, len=" << rxFrame.size();
	if (rxFrame.size() >= 5) {
		switch (rxFrame[1]) {
			case RESP_ADC_READINGS: {
				TFrame_ADC_readings rx;
				::memcpy((uint8_t *)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewADCCallback(rx.payload);
			} break;

			case RESP_ENCODER_READINGS: {
				TFrame_ENCODERS_readings rx;
				::memcpy((uint8_t *)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewENCCallback(rx.payload);
			} break;

			case RESP_EMS22A_READINGS: {
				TFrame_ENCODER_ABS_reading rx;
				::memcpy((uint8_t *)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewENCAbsCallback(rx.payload);
			} break;

			case RESP_CONTROL_SIGNAL: {
				TFrame_CONTROL_SIGNAL rx;
				::memcpy((uint8_t *)&rx, &rxFrame[0], sizeof(rx));
				daqOnNewControlSignalCallback(rx.payload);
			} break;
		};
	}
}

bool VehicleControllerLowLevel::iterate() {
	// Housekeeping -------
	// Controller mode changed?
	if (m_modes_changed) {
		m_modes_changed = false;
		TFrameCMD_CONTROL_MODE cmd;
		cmd.payload.steer_enable = !m_mode_openloop_steer;
		cmd.payload.throttle_enable = !m_mode_openloop_throttle;
		cmd.calc_and_update_checksum();
		WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
		ROS_INFO("Sending new controller modes: STEER=%s THROTTLE=%s", m_mode_openloop_steer ? "MANUAL" : "AUTO", m_mode_openloop_throttle ? "MANUAL" : "AUTO");
	}

	// New joystick
	if (!m_autonomous_driving_mode && m_joy_changed) {
		// X: Steering
		if (m_mode_openloop_steer) {
			TFrameCMD_OPENLOOP_STEERING_SETPOINT cmd;
			cmd.payload.SETPOINT_OPENLOOP_STEER_SPEED = m_joy_x * 255.0;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));

			ROS_INFO("Sending openloop STEER: %f", m_joy_x);
		} else {
			MRPT_TODO("Recalibrate steer pos range");
			int16_t steer_pos = 512 * m_joy_x;

			TFrameCMD_CONTROL_STEERING_SETPOINT cmd;
			cmd.payload.SETPOINT_STEER_POS = steer_pos;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));

			ROS_INFO("Sending closedloop STEER: %d", steer_pos);
		}

		// Y: throttle
		if (m_mode_openloop_throttle) {
			TFrameCMD_OPENLOOP_THROTTLE_SETPOINT cmd;
			cmd.payload.SETPOINT_OPENLOOP_THROTTLE = m_joy_y;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));

			ROS_INFO("Sending openloop THROTTLE: %f", m_joy_y);
		} else {
			const float MAX_VEL_MPS = 2.0;
			float vel_mps = m_joy_y * MAX_VEL_MPS;

			TFrameCMD_CONTROL_THROTTLE_SETPOINT cmd;
			cmd.payload.SETPOINT_CONTROL_THROTTLE_SPEED = vel_mps;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));

			ROS_INFO("Sending closedloop THROTTLE: %.03f m/s", vel_mps);
		}
	}

	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;

	if (!m_serial.isOpen()) {
		if (!this->initialize())
			return false;
	}

	std::vector<uint8_t> rxFrame;
	while (ReceiveFrameFromController(rxFrame) && ++nFrames < MAX_FRAMES_PER_ITERATE) {
		// Process them:
		processIncommingFrame(rxFrame);
	}

	// if no frame was received, ping the uC to keep comms alive:
	if (!nFrames && m_NOP_sent_counter++ > 20) {
		m_NOP_sent_counter = 0;

		// Send a dummy NOP command
		TFrameCMD_NOP cmd;
		cmd.calc_and_update_checksum();
		return WriteBinaryFrame(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
	}

	return true;
}

void VehicleControllerLowLevel::autonomousModeCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_autonomous_driving_mode = msg->data;
}

void VehicleControllerLowLevel::modeSteeringCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_mode_openloop_steer = msg->data;
}
void VehicleControllerLowLevel::modeThrottleCallback(const std_msgs::Bool::ConstPtr &msg) {
	m_mode_openloop_throttle = msg->data;
}

void VehicleControllerLowLevel::ejexCallback(const std_msgs::Float64::ConstPtr &msg) {
	m_joy_x = msg->data;
	m_joy_changed = true;
}

void VehicleControllerLowLevel::ejeyCallback(const std_msgs::Float64::ConstPtr &msg) {
	m_joy_y = msg->data;
	m_joy_changed = true;
}

void VehicleControllerLowLevel::daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data) {
	ual_ecar_vehicle_controller::AnalogReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	const int N = sizeof(data.adc_data) / sizeof(data.adc_data[0]);
	msg.adc_data.resize(N);
	for (int i = 0; i < N; i++)
		msg.adc_data[i] = data.adc_data[i];

	m_pub_ADC.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data) {
	ual_ecar_vehicle_controller::EncodersReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	msg.period_ms = data.period_ms_tenths;
	const int N = sizeof(data.encoders) / sizeof(data.encoders[0]);

	msg.encoder_values.resize(N);
	for (int i = 0; i < N; i++)
		msg.encoder_values[i] = data.encoders[i];

	m_pub_ENC.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewENCAbsCallback(const TFrame_ENCODER_ABS_reading_payload_t &data) {
	ual_ecar_vehicle_controller::EncoderAbsReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	msg.encoder_status = data.enc_status;
	msg.encoder_value = data.enc_pos;

	m_pub_ENC_ABS.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewControlSignalCallback(const TFrame_CONTROL_SIGNAL_payload_t &data) {
	ual_ecar_vehicle_controller::ControlSignal msg;

	msg.timestamp_ms = data.timestamp_ms_tenth;
	msg.Steer_controller_signal = data.Steer_control_signal;
	msg.Throttle_controller_signal = data.Throttle_control_signal;

	m_pub_Control_signal.publish(msg);
}

bool VehicleControllerLowLevel::AttemptConnection() {
	if (m_serial.isOpen())
	return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100, 0, 10, 0, 50);

		MRPT_LOG_INFO_FMT("[VehicleControllerLowLevel::AttemptConnection] Serial port '%s' open was successful.",m_serial_port_name.c_str());
		return true;
	} catch (std::exception &e) {
		MRPT_LOG_ERROR_FMT("[VehicleControllerLowLevel::AttemptConnection] COMMS error: %s",e.what());
		return false;
	}
}

/** Sends a binary packet (returns false on COMMS error) */
bool VehicleControllerLowLevel::WriteBinaryFrame(const uint8_t *full_frame,const size_t full_frame_len) {
	if (!AttemptConnection())
		return false;

	ASSERT_(full_frame != NULL);

	try {
#ifdef DEBUG_TRACES
	{
		std::string s;
		s += mrpt::format("TX frame (%u bytes): ", (unsigned int)full_frame_len);
		for (size_t i = 0; i < full_frame_len; i++)
			s += mrpt::format("%02X ", full_frame[i]);
		MRPT_LOG_INFO_FMT("Tx frame: %s", s.c_str());
	}
#endif

#if MRPT_VERSION >= 0x199
	m_serial.Write(full_frame, full_frame_len);
#else
	m_serial.WriteBuffer(full_frame, full_frame_len);
#endif
	return true;
	} catch (std::exception &) {
		return false;
	}
}

bool VehicleControllerLowLevel::SendFrameAndWaitAnswer(const uint8_t *full_frame, const size_t full_frame_len, const int num_retries, const int retries_interval_ms, uint8_t expected_ans_opcode) {
	if (expected_ans_opcode == 0 && full_frame_len > 2)
		expected_ans_opcode = full_frame[1] + 0x70; // answer OPCODE convention

	for (int iter = 0; iter < num_retries; iter++) {
		if (iter > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(retries_interval_ms));
		// Send:
		if (!WriteBinaryFrame(full_frame, full_frame_len))
			continue;

		// Wait for answer:
		std::vector<uint8_t> rxFrame;
		if (this->ReceiveFrameFromController(rxFrame) && rxFrame.size() > 4) {
			const auto RX_OPCODE = rxFrame[1];
			if (RX_OPCODE == expected_ans_opcode) {
				// We received the ACK from the uC, yay!
				MRPT_LOG_INFO_FMT("SendFrameAndWaitAnswer(): Rx ACK for OPCODE=0x%02X after %i retries.", full_frame_len > 2 ? full_frame[1] : 0, iter);
				return true;
			} else {
				// Ensure the frame gets processed:
				processIncommingFrame(rxFrame);
			}
		}
	}
	MRPT_LOG_ERROR_FMT("SendFrameAndWaitAnswer(): Missed ACK for OPCODE=0x%02X", full_frame_len > 2 ? full_frame[1] : 0);
	return false; // No answer!
}

bool VehicleControllerLowLevel::ReceiveFrameFromController(std::vector<uint8_t> &rxFrame) {
	rxFrame.clear();
	size_t nFrameBytes = 0;
	std::vector<uint8_t> buf;
	buf.resize(0x10000);
	buf[0] = buf[1] = 0;

	size_t lengthField;

	/*
	START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    |
	END_FLAG | 0x69          1 byte      1 byte       N bytes       =sum(data)
	0x96
	*/

	//                                   START_FLAG     OPCODE + LEN       DATA
	//                                   CHECKSUM +  END_FLAG
	while (nFrameBytes < (lengthField = (1 + 1 + 1 + buf[2] + 1 + 1))) {
	if (lengthField > 200) {
		nFrameBytes = 0; // No es cabecera de trama correcta
		buf[1] = buf[2] = 0;
		MRPT_LOG_INFO("[rx] Reset frame (invalid len)");
	}

	size_t nBytesToRead;
	if (nFrameBytes < 3)
		nBytesToRead = 1;
	else
		nBytesToRead = (lengthField)-nFrameBytes;

	size_t nRead;
	try {
		nRead = m_serial.Read(&buf[0] + nFrameBytes, nBytesToRead);
	} catch (std::exception &e) {
		// Disconnected?
		MRPT_LOG_ERROR_FMT("ReceiveFrameFromController(): Comms error: %s", e.what());
		return false;
	}

	if (!nRead && !nFrameBytes) {
		// cout << "[rx] No frame (buffer empty)\n";
		return false;
	}

	if (nRead < nBytesToRead) {
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(1ms);
	}

	// Reading was OK:
	// Check start flag:
	bool is_ok = true;

	if (!nFrameBytes && buf[0] != FRAME_START_FLAG) {
		is_ok = false;
		MRPT_LOG_INFO("[rx] Reset frame (start flag)");
	}

	if (nFrameBytes > 2 && nFrameBytes + nRead == lengthField) {
		if (buf[nFrameBytes + nRead - 1] != FRAME_END_FLAG) {
			is_ok = false;
			MRPT_LOG_INFO("[rx] Reset frame (end flag)");
		}
	}

	MRPT_TODO("Checksum");

	if (is_ok) {
		nFrameBytes += nRead;
	} else {
		nFrameBytes = 0; // No es cabecera de trama correcta
		buf[1] = buf[2] = 0;
	}
	}

	// Frame received
	lengthField = buf[2] + 5;
	rxFrame.resize(lengthField);
	::memcpy(&rxFrame[0], &buf[0], lengthField);

#ifdef DEBUG_TRACES
{
	std::string s;
	s += mrpt::format("RX frame (%u bytes): ", (unsigned int)lengthField);
	for (size_t i = 0; i < lengthField; i++)
	s += mrpt::format("%02X ", rxFrame[i]);
	MRPT_LOG_INFO_FMT("%s", s.c_str());
}
#endif

	// All OK
	return true;
}

bool VehicleControllerLowLevel::CMD_GPIO_output(int pin, bool pinState) {
	TFrameCMD_GPIO_output cmd;
	cmd.payload.pin_index = pin;
	cmd.payload.pin_value = pinState ? 1 : 0;

	cmd.calc_and_update_checksum();

	return SendFrameAndWaitAnswer(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
}

//!< Sets the clutch
bool VehicleControllerLowLevel::CMD_DAC(int dac_index, double dac_value_volts) {
	uint16_t dac_counts = 4096 * dac_value_volts / 5.0;
	saturate(dac_counts, uint16_t(0), uint16_t(4095));

	TFrameCMD_SetDAC cmd;
	cmd.payload.dac_index = dac_index;
	cmd.payload.dac_value_HI = dac_counts >> 8;
	cmd.payload.dac_value_LO = dac_counts & 0x00ff;
	cmd.payload.flag_enable_timeout = true;

	cmd.calc_and_update_checksum();

	return SendFrameAndWaitAnswer(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
}

bool VehicleControllerLowLevel::IsConnected() const {
	return m_serial.isOpen();
}
