/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |               Copyright (C) 2014-17  University of Almeria                |
   +---------------------------------------------------------------------------+
 */

#include <array>
#include <cstring>
#include <iostream>
#include <ros/console.h>
#include <thread>
#include <ual_ecar_vehicle_controller/AnalogReading.h>
#include <ual_ecar_vehicle_controller/ControlSignalSteer.h>
#include <ual_ecar_vehicle_controller/ControlSignalSpeedCruise.h>
#include <ual_ecar_vehicle_controller/EncoderAbsReading.h>
#include <ual_ecar_vehicle_controller/EncodersReading.h>
#include <ual_ecar_vehicle_controller/SteerControllerStatus.h>
#include <ual_ecar_vehicle_controller/VehicleControllerLowLevel.h>
#include <ual_ecar_vehicle_controller/vehicle_controller2pc-structs.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
using mrpt::saturate;
#else
using mrpt::utils::saturate;
#endif

// Define to see serial communication traces
#define DEBUG_TRACES

bool VehicleControllerLowLevel::initialize()
{
	ROS_INFO("VehicleControllerLowLevel::inicialize() ok.");

	m_serial_Steer_port_name =
		"/dev/serial/by-id/"
		"usb-Ual-ARM-eCARM_Claraquino_eCARM_2018_03_06_FT3ALM93-if00-port0";
	m_serial_Steer_port_baudrate = 500000;
	m_nh_params.getParam("STEER_SERIAL_PORT", m_serial_Steer_port_name);
	m_nh_params.getParam(
		"STEER_SERIAL_PORT_BAUDRATE", m_serial_Steer_port_baudrate);

	m_serial_SpeedCruise_port_name =
		"/dev/serial/by-id/"
		"usb-Ual-ARM-eCARM_Claraquino_eCARM_FT37TV91-if00-port0";
	m_serial_SpeedCruise_port_baudrate = 500000;
	m_nh_params.getParam(
		"SPEEDCRUISE_SERIAL_PORT", m_serial_SpeedCruise_port_name);
	m_nh_params.getParam(
		"SPEEDCRUISE_SERIAL_PORT_BAUDRATE", m_serial_SpeedCruise_port_baudrate);

	// Try to connect to Steer Claraquino
	if (this->AttemptConnection(m_serial_Steer))
	{
		ROS_INFO(
			"Connection OK to VehicleLowLevelController Steer [Claraquino].");
	}
	else
	{
		ROS_ERROR(
			"Error in VehicleControllerLowLevel::AttemptConnection()! Steer");
		return false;
	}
	// Try to connect to SpeerCruise Claraquino
	/*	if (this->AttemptConnection(m_serial_SpeedCruise))
		{
			ROS_INFO("Connection OK to VehicleLowLevelController
	   SpeedCruise[Claraquino].");
		}
		else
		{
			ROS_ERROR("Error in VehicleControllerLowLevel::AttemptConnection()!
	   SpeedCruise"); return false;
		}
	*/
	// Publisher: Controller Status
	m_pub_controller_status =
		m_nh.advertise<ual_ecar_vehicle_controller::SteerControllerStatus>(
			"vehicle_controller_status", 10);
	// Publisher: ADC data.
	m_pub_Steer_ADC =
		m_nh.advertise<ual_ecar_vehicle_controller::AnalogReading>(
			"claraquino_steer_adc", 10);
	m_pub_SpeedCruise_ADC =
		m_nh.advertise<ual_ecar_vehicle_controller::AnalogReading>(
			"claraquino_speedcruise_adc", 10);
	// Publisher: ENC data
	m_pub_Steer_ENC =
		m_nh.advertise<ual_ecar_vehicle_controller::EncodersReading>(
			"claraquino_steer_encoders", 10);
	m_pub_SpeedCruise_ENC =
		m_nh.advertise<ual_ecar_vehicle_controller::EncodersReading>(
			"claraquino_speedcruise_encoders", 10);
	// Publisher: ABS ENC data
	m_pub_ENC_ABS =
		m_nh.advertise<ual_ecar_vehicle_controller::EncoderAbsReading>(
			"claraquino_abs_encoder", 10);
	// Publisher: Control signal data
	m_pub_Steer_Control_signal =
		m_nh.advertise<ual_ecar_vehicle_controller::ControlSignalSteer>(
			"claraquino_steer_control_signal", 10);
	m_pub_SpeedCruise_Control_signal =
		m_nh.advertise<ual_ecar_vehicle_controller::ControlSignalSpeedCruise>(
			"claraquino_speedcruise_control_signal", 10);

	// Subscriber:
	m_sub_contr_status[0] = m_nh.subscribe(
		"vehicle_openloop_mode_steering", 10,
		&VehicleControllerLowLevel::modeSteeringCallback, this);
	m_sub_contr_status[1] = m_nh.subscribe(
		"vehicle_openloop_mode_throttle", 10,
		&VehicleControllerLowLevel::modeThrottleCallback, this);
	m_sub_eje_x = m_nh.subscribe(
		"joystick_eje_x", 10, &VehicleControllerLowLevel::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe(
		"joystick_eje_y", 10, &VehicleControllerLowLevel::ejeyCallback, this);
	m_sub_autonomous_driving = m_nh.subscribe(
		"vehicle_autonomous_mode", 10,
		&VehicleControllerLowLevel::autonomousModeCallback, this);
	m_sub_brake_enable = m_nh.subscribe(
		"vehicle_brake_enable", 10,
		&VehicleControllerLowLevel::brakeenableCallback, this);
	/*Sub:System_Identification[Controller & Smith predictor params,
	   Feedforwards...]
	*/
	// Decimation params
	{
		TFrameCMD_VERBOSITY_CONTROL_payload_t Decimation_config_Steer,
			Decimation_config_SpeedCruise;
		int decimate_ADC_Steer = 10, decimate_ENCABS_Steer = 10,
			decimate_CPU_Steer = 10000, decimate_CONTROLSIGNAL_Steer = 10,
			decimate_ENCINC_Steer = 10;

		m_nh_params.getParam("DECIM_ADC_STEER", decimate_ADC_Steer);
		m_nh_params.getParam("DECIM_ENCABS_STEER", decimate_ENCABS_Steer);
		m_nh_params.getParam("DECIM_ENCINC_STEER", decimate_ENCINC_Steer);
		m_nh_params.getParam("DECIM_CPU_STEER", decimate_CPU_Steer);
		m_nh_params.getParam(
			"DECIM_CONTROLSIGNAL_STEER", decimate_CONTROLSIGNAL_Steer);

		if (decimate_ADC_Steer > 0 && decimate_ENCABS_Steer > 0 &&
			decimate_CPU_Steer > 0 && decimate_CONTROLSIGNAL_Steer > 0 &&
			decimate_ENCINC_Steer > 0)
		{
			Decimation_config_Steer.decimate_ADC = decimate_ADC_Steer;
			Decimation_config_Steer.decimate_ENCABS = decimate_ENCABS_Steer;
			Decimation_config_Steer.decimate_ENCINC = decimate_ENCINC_Steer;
			Decimation_config_Steer.decimate_CPU = decimate_CPU_Steer;
			Decimation_config_Steer.decimate_CONTROLSIGNAL =
				decimate_CONTROLSIGNAL_Steer;

			MRPT_LOG_INFO_FMT(
				" Steer Decimation: ADC=%i  ENCABS=%i  ENCINC=%i  CPU=%i  "
				"Control Signal=%i",
				decimate_ADC_Steer, decimate_ENCABS_Steer,
				decimate_ENCINC_Steer, decimate_CPU_Steer,
				decimate_CONTROLSIGNAL_Steer);
			this->CMD_Decimation_configuration(
				Decimation_config_Steer, m_serial_Steer);
		}

		int decimate_ADC_SpeedCruise = 10, decimate_CPU_SpeedCruise = 10000,
			decimate_CONTROLSIGNAL_SpeedCruise = 10,
			decimate_ENCINC_SpeedCruise = 10;

		m_nh_params.getParam("DECIM_ADC_SPEEDCRUISE", decimate_ADC_SpeedCruise);
		m_nh_params.getParam(
			"DECIM_ENCINC_SPEEDCRUISE", decimate_ENCINC_SpeedCruise);
		m_nh_params.getParam("DECIM_CPU_SPEEDCRUISE", decimate_CPU_SpeedCruise);
		m_nh_params.getParam(
			"DECIM_CONTROLSIGNAL_SPEEDCRUISE",
			decimate_CONTROLSIGNAL_SpeedCruise);

		if (decimate_ADC_SpeedCruise > 0 && decimate_CPU_SpeedCruise > 0 &&
			decimate_CONTROLSIGNAL_SpeedCruise > 0 &&
			decimate_ENCINC_SpeedCruise > 0)
		{
			Decimation_config_SpeedCruise.decimate_ADC =
				decimate_ADC_SpeedCruise;
			Decimation_config_SpeedCruise.decimate_ENCINC =
				decimate_ENCINC_SpeedCruise;
			Decimation_config_SpeedCruise.decimate_CPU =
				decimate_CPU_SpeedCruise;
			Decimation_config_SpeedCruise.decimate_CONTROLSIGNAL =
				decimate_CONTROLSIGNAL_SpeedCruise;

			MRPT_LOG_INFO_FMT(
				" Speed Cruise Decimation: ADC=%i  ENCINC=%i  "
				"CPU=%i  Control Signal=%i",
				decimate_ADC_SpeedCruise, decimate_ENCINC_SpeedCruise,
				decimate_CPU_SpeedCruise, decimate_CONTROLSIGNAL_SpeedCruise);
			this->CMD_Decimation_configuration(
				Decimation_config_SpeedCruise, m_serial_SpeedCruise);
		}
	}
	return true;
}

void VehicleControllerLowLevel::processIncommingFrame(
	const std::vector<uint8_t>& rxFrame, CSerialPort& m_serial)
{
	// MRPT_LOG_INFO_STREAM  << "Rx frame, len=" << rxFrame.size();
	if (&m_serial == &m_serial_Steer)
	{
		if (rxFrame.size() >= 5)
		{
			switch (rxFrame[1])
			{
				case RESP_ADC_READINGS:
				{
					TFrame_ADC_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewADCCallback(rx.payload, m_serial);
				}
				break;

				case RESP_EMS22A_READINGS:
				{
					TFrame_ENCODER_ABS_reading rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewENCAbsCallback(rx.payload);
				}
				break;

				case RESP_ENCODER_READINGS:
				{
					TFrame_ENCODERS_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewENCCallback(rx.payload, m_serial);
				}
				break;

				case RESP_CONTROL_SIGNAL:
				{
					TFrame_STEER_CONTROL_SIGNAL rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewSteerControlSignalCallback(rx.payload);
				}
				break;
			};
		}
	}
	if (&m_serial == &m_serial_SpeedCruise)
	{
		if (rxFrame.size() >= 5)
		{
			switch (rxFrame[1])
			{
				case RESP_ADC_READINGS:
				{
					TFrame_ADC_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewADCCallback(rx.payload, m_serial);
				}
				break;

				case RESP_ENCODER_READINGS:
				{
					TFrame_ENCODERS_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewENCCallback(rx.payload, m_serial);
				}
				break;

				case RESP_CONTROL_SIGNAL:
				{
					TFrame_SPEEDCRUISE_CONTROL_SIGNAL rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));
					daqOnNewSpeedCruiseControlSignalCallback(rx.payload);
				}
				break;
			};
		}
	}
}

bool VehicleControllerLowLevel::iterate()
{
	// Housekeeping -------
	// TO-DO: setSteer_ControllerParams
	// TO-DO: setThrottle_ControllerParams
	// TO-DO: setBrake_ControllerParams

	// Controller mode changed?
	if (m_mode_steer_changed)
	{
		m_mode_steer_changed = false;
		TFrameCMD_STEER_CONTROL_MODE cmd;
		cmd.payload.steer_enable = !m_mode_openloop_steer;
		cmd.calc_and_update_checksum();
		WriteBinaryFrame(
			reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial_Steer);
		ROS_INFO_THROTTLE(
			1, "Sending new STEER controller mode: %s",
			m_mode_openloop_steer ? "MANUAL" : "AUTO");
	}
	if (m_mode_throttle_changed || m_mode_brake_changed)
	{
		m_mode_throttle_changed = false;
		m_mode_brake_changed = false;
		TFrameCMD_SPEEDCRUISE_CONTROL_MODE cmd;
		cmd.payload.throttle_enable = !m_mode_openloop_throttle;
		cmd.payload.brake_enable = !m_mode_brake_enable;
		cmd.calc_and_update_checksum();
		WriteBinaryFrame(
			reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd),
			m_serial_SpeedCruise);
		ROS_INFO_THROTTLE(
			1,
			"Sending new SPEED CRUISE controller mode: THROTTLE: %s BRAKE: %s",
			m_mode_openloop_throttle ? "MANUAL" : "AUTO",
			m_mode_brake_enable ? "MANUAL" : "AUTO");
	}
	// New joystick
	if (!m_autonomous_driving_mode && m_joy_changed)
	{
		// X: Steering
		if (m_mode_openloop_steer)
		{
			TFrameCMD_OPENLOOP_STEERING_SETPOINT cmd;
			cmd.payload.SETPOINT_OPENLOOP_STEER_SPEED = m_joy_x * 255.0;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(
				reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial_Steer);
			ROS_INFO_THROTTLE(
				1, "Sending openloop STEER: %d",
				cmd.payload.SETPOINT_OPENLOOP_STEER_SPEED);
		}
		else
		{
			MRPT_TODO("Recalibrate steer pos range");
			TFrameCMD_CONTROL_STEERING_SETPOINT cmd;
			cmd.payload.SETPOINT_STEER_POS = 512 * m_joy_x;
			cmd.calc_and_update_checksum();
			WriteBinaryFrame(
				reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial_Steer);
			ROS_INFO_THROTTLE(
				1, "Sending closedloop STEER: %d",
				cmd.payload.SETPOINT_STEER_POS);
		}
		// Y: throttle
		if (!m_mode_brake_enable)
		{
			if (m_mode_openloop_throttle)
			{
				TFrameCMD_OPENLOOP_THROTTLE_SETPOINT cmd;
				cmd.payload.SETPOINT_OPENLOOP_THROTTLE = m_joy_y;
				cmd.calc_and_update_checksum();
				WriteBinaryFrame(
					reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd),
					m_serial_SpeedCruise);
				ROS_INFO_THROTTLE(
					1, "Sending openloop THROTTLE: %f",
					cmd.payload.SETPOINT_OPENLOOP_THROTTLE);
			}
			else
			{
				const float MAX_VEL_MPS = 2.0;
				float vel_mps = m_joy_y * MAX_VEL_MPS;
				// TO-DO: CONTROL_BRAKE_SETPOINT
				TFrameCMD_CONTROL_THROTTLE_SETPOINT cmd;
				cmd.payload.SETPOINT_CONTROL_THROTTLE_SPEED = vel_mps;
				cmd.calc_and_update_checksum();
				WriteBinaryFrame(
					reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd),
					m_serial_SpeedCruise);
				ROS_INFO_THROTTLE(
					1, "Sending closedloop THROTTLE: %.03f m/s", vel_mps);
			}
		}
		// Brake:
		{
			TFrameCMD_OPENLOOP_BRAKE_SETPOINT cmd_brake;
			if (m_mode_brake_enable)
			{
				cmd_brake.payload.SETPOINT_OPENLOOP_BRAKE =
					m_joy_y * 255.0 * 0.5;
			}
			else
			{
				cmd_brake.payload.SETPOINT_OPENLOOP_BRAKE = 0;
			}
			cmd_brake.calc_and_update_checksum();
			WriteBinaryFrame(
				reinterpret_cast<uint8_t*>(&cmd_brake), sizeof(cmd_brake),
				m_serial_SpeedCruise);
			ROS_INFO_THROTTLE(
				1, "Sending openloop Brake: %d",
				cmd_brake.payload.SETPOINT_OPENLOOP_BRAKE);
		}
	}

	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;
	// STEER
	if (!m_serial_Steer.isOpen())
	{
		if (!this->initialize()) return false;
	}
	std::vector<uint8_t> rxFrame;
	while (ReceiveFrameFromController(rxFrame, m_serial_Steer) &&
		   ++nFrames < MAX_FRAMES_PER_ITERATE)
	{
		// Process them:
		processIncommingFrame(rxFrame, m_serial_Steer);
	}
	// if no frame was received, ping the uC to keep comms alive:
	if (!nFrames && m_NOP_sent_counter++ > 20)
	{
		m_NOP_sent_counter = 0;
		// Send a dummy NOP command
		TFrameCMD_NOP cmd;
		cmd.calc_and_update_checksum();
		return WriteBinaryFrame(
			reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial_Steer);
	}
	// SPEED CRUISE
	nFrames = 0;
	if (!m_serial_SpeedCruise.isOpen())
	{
		if (!this->initialize()) return false;
	}
	std::vector<uint8_t> rxFrame_SpeedCruise;
	while (
		ReceiveFrameFromController(rxFrame_SpeedCruise, m_serial_SpeedCruise) &&
		++nFrames < MAX_FRAMES_PER_ITERATE)
	{
		// Process them:
		processIncommingFrame(rxFrame_SpeedCruise, m_serial_SpeedCruise);
	}
	// if no frame was received, ping the uC to keep comms alive:
	if (!nFrames && m_NOP_sent_counter++ > 20)
	{
		m_NOP_sent_counter = 0;
		// Send a dummy NOP command
		TFrameCMD_NOP cmd;
		cmd.calc_and_update_checksum();
		return WriteBinaryFrame(
			reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial_Steer);
	}
	return true;
}

void VehicleControllerLowLevel::autonomousModeCallback(
	const std_msgs::Bool::ConstPtr& msg)
{
	m_autonomous_driving_mode = msg->data;
	m_modes_changed = true;
}

void VehicleControllerLowLevel::modeSteeringCallback(
	const std_msgs::Bool::ConstPtr& msg)
{
	m_mode_openloop_steer = msg->data;
	m_mode_steer_changed = true;
}

void VehicleControllerLowLevel::modeThrottleCallback(
	const std_msgs::Bool::ConstPtr& msg)
{
	m_mode_openloop_throttle = msg->data;
	m_mode_throttle_changed = true;
}

void VehicleControllerLowLevel::brakeenableCallback(
	const std_msgs::Bool::ConstPtr& msg)
{
	m_mode_brake_enable = msg->data;
	m_mode_brake_changed = true;
}

void VehicleControllerLowLevel::ejexCallback(
	const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_x = msg->data;
	m_joy_changed = true;
}

void VehicleControllerLowLevel::ejeyCallback(
	const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_y = msg->data;
	m_joy_changed = true;
}

void VehicleControllerLowLevel::daqOnNewADCCallback(
	const TFrame_ADC_readings_payload_t& data, CSerialPort& m_serial)
{
	ual_ecar_vehicle_controller::AnalogReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	const int N = sizeof(data.adc_data) / sizeof(data.adc_data[0]);
	msg.adc_data.resize(N);
	for (int i = 0; i < N; i++) msg.adc_data[i] = data.adc_data[i];

	((&m_serial == &m_serial_Steer) ? m_pub_Steer_ADC : m_pub_SpeedCruise_ADC)
		.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewENCCallback(
	const TFrame_ENCODERS_readings_payload_t& data, CSerialPort& m_serial)
{
	ual_ecar_vehicle_controller::EncodersReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	msg.period_ms = data.period_ms_tenths;
	const int N = sizeof(data.encoders) / sizeof(data.encoders[0]);

	msg.encoder_values.resize(N);
	for (int i = 0; i < N; i++) msg.encoder_values[i] = data.encoders[i];

	((&m_serial == &m_serial_Steer) ? m_pub_Steer_ENC : m_pub_SpeedCruise_ENC)
		.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewENCAbsCallback(
	const TFrame_ENCODER_ABS_reading_payload_t& data)
{
	ual_ecar_vehicle_controller::EncoderAbsReading msg;

	msg.timestamp_ms = data.timestamp_ms_tenths;
	msg.encoder_status = data.enc_status;
	msg.encoder_value = data.enc_pos;

	m_pub_ENC_ABS.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewSteerControlSignalCallback(
	const TFrame_STEER_CONTROL_SIGNAL_payload_t& data)
{
	ual_ecar_vehicle_controller::ControlSignalSteer msg;

	msg.timestamp_ms = data.timestamp_ms_tenth;
	msg.Steer_controller_signal = data.Steer_control_signal;
	msg.Encoder_Absoluto = data.Encoder_absoluto;
	msg.Encoder_Incremental = data.Encoder_incremental;
	msg.Steer_ADC_current_sense = data.Steer_ADC_current_sense * 5.0 / 1023.0;
	msg.Encoder_controller_signal = data.Encoder_signal;
	msg.steer_mech_limit_reached = data.steer_mech_limit_reached;
	msg.enc_offset_correction = data.enc_offset_correction;

	m_pub_Steer_Control_signal.publish(msg);
}

void VehicleControllerLowLevel::daqOnNewSpeedCruiseControlSignalCallback(
	const TFrame_SPEEDCRUISE_CONTROL_SIGNAL_payload_t& data)
{
	ual_ecar_vehicle_controller::ControlSignalSpeedCruise msg;

	msg.timestamp_ms = data.timestamp_ms_tenth;
	msg.Brake_controller_signal = data.Brake_control_signal;
	msg.Throttle_controller_signal = data.Throttle_control_signal;
	msg.Encoder_Incremental = data.Brake_Encoder_incremental;
	// msg.VehSpeed_feedback_phidgets = data.VehSpeed_feedback;
	msg.Brake_ADC_current_sense = data.Brake_ADC_current_sense * 5.0 / 1023.0;
	msg.Throttle_analog_feedback = data.Throttle_analog_feedback * 5.0 / 1023.0;

	m_pub_SpeedCruise_Control_signal.publish(msg);
}

bool VehicleControllerLowLevel::AttemptConnection(CSerialPort& m_serial)
{
	if (m_serial.isOpen()) return true;  // Already open.

	try
	{
		const auto portname = (&m_serial == &m_serial_Steer)
								  ? m_serial_Steer_port_name
								  : m_serial_SpeedCruise_port_name;
		const auto portbaud = (&m_serial == &m_serial_Steer)
								  ? m_serial_Steer_port_baudrate
								  : m_serial_SpeedCruise_port_baudrate;

		m_serial.open(portname);

		// Set basic params:
		m_serial.setConfig(portbaud);
		m_serial.setTimeouts(100, 0, 10, 0, 50);

		MRPT_LOG_INFO_FMT(
			"[VehicleControllerLowLevel::AttemptConnection] Serial port '%s' "
			"open was successful.",
			portname.c_str());
		return true;
	}
	catch (std::exception& e)
	{
		MRPT_LOG_ERROR_FMT(
			"[VehicleControllerLowLevel::AttemptConnection] COMMS error: %s",
			e.what());
		return false;
	}
}

/** Sends a binary packet (returns false on COMMS error) */
bool VehicleControllerLowLevel::WriteBinaryFrame(
	const uint8_t* full_frame, const size_t full_frame_len,
	CSerialPort& m_serial)
{
	if (!AttemptConnection(m_serial)) return false;

	ASSERT_(full_frame != NULL);

	try
	{
#ifdef DEBUG_TRACES
		{
			std::string s;
			s += mrpt::format(
				"TX frame (%u bytes): ", (unsigned int)full_frame_len);
			for (size_t i = 0; i < full_frame_len; i++)
				s += mrpt::format("%02X ", full_frame[i]);
			if (&m_serial == &m_serial_Steer)
			{
				MRPT_LOG_INFO_FMT("Tx frame: %s. STEER_SERIAL_PORT", s.c_str());
			}
			else
			{
				MRPT_LOG_INFO_FMT(
					"Tx frame: %s. SPEEDCRUISE_SERIAL_PORT", s.c_str());
			}
		}
#endif

#if MRPT_VERSION >= 0x199
		m_serial.Write(full_frame, full_frame_len);
#else
		m_serial.WriteBuffer(full_frame, full_frame_len);
#endif
		return true;
	}
	catch (std::exception&)
	{
		return false;
	}
}

bool VehicleControllerLowLevel::SendFrameAndWaitAnswer(
	const uint8_t* full_frame, const size_t full_frame_len,
	CSerialPort& m_serial, const int num_retries, const int retries_interval_ms,
	uint8_t expected_ans_opcode)
{
	if (expected_ans_opcode == 0 && full_frame_len > 2)
		expected_ans_opcode = full_frame[1] + 0x70;  // answer OPCODE convention

	for (int iter = 0; iter < num_retries; iter++)
	{
		if (iter > 0)
			std::this_thread::sleep_for(
				std::chrono::milliseconds(retries_interval_ms));
		// Send:
		if (!WriteBinaryFrame(full_frame, full_frame_len, m_serial)) continue;

		// Wait for answer:
		std::vector<uint8_t> rxFrame;
		// Cambiar m_serial
		if (this->ReceiveFrameFromController(rxFrame, m_serial) &&
			rxFrame.size() > 4)
		{
			const auto RX_OPCODE = rxFrame[1];
			if (RX_OPCODE == expected_ans_opcode)
			{
				// We received the ACK from the uC, yay!
				MRPT_LOG_INFO_FMT(
					"SendFrameAndWaitAnswer(): Rx ACK for OPCODE=0x%02X after "
					"%i retries.",
					full_frame_len > 2 ? full_frame[1] : 0, iter);
				return true;
			}
			else
			{
				// Ensure the frame gets processed:
				processIncommingFrame(rxFrame, m_serial);
			}
		}
	}
	MRPT_LOG_ERROR_FMT(
		"SendFrameAndWaitAnswer(): Missed ACK for OPCODE=0x%02X",
		full_frame_len > 2 ? full_frame[1] : 0);
	return false;  // No answer!
}

bool VehicleControllerLowLevel::ReceiveFrameFromController(
	std::vector<uint8_t>& rxFrame, CSerialPort& m_serial)
{
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
	while (nFrameBytes < (lengthField = (1 + 1 + 1 + buf[2] + 1 + 1)))
	{
		if (lengthField > 200)
		{
			nFrameBytes = 0;  // No es cabecera de trama correcta
			buf[1] = buf[2] = 0;
			MRPT_LOG_INFO("[rx] Reset frame (invalid len)");
		}

		size_t nBytesToRead;
		if (nFrameBytes < 3)
			nBytesToRead = 1;
		else
			nBytesToRead = (lengthField)-nFrameBytes;

		size_t nRead;
		try
		{
			nRead = m_serial.Read(&buf[0] + nFrameBytes, nBytesToRead);
		}
		catch (std::exception& e)
		{
			// Disconnected?
			MRPT_LOG_ERROR_FMT(
				"ReceiveFrameFromController(): Comms error: %s", e.what());
			return false;
		}

		if (!nRead && !nFrameBytes)
		{
			// cout << "[rx] No frame (buffer empty)\n";
			return false;
		}

		if (nRead < nBytesToRead)
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1ms);
		}

		// Reading was OK:
		// Check start flag:
		bool is_ok = true;

		if (!nFrameBytes && buf[0] != FRAME_START_FLAG)
		{
			is_ok = false;
			if (&m_serial == &m_serial_Steer)
			{
				MRPT_LOG_INFO(
					"[rx] Reset frame (start flag). STEER_SERIAL_PORT");
			}
			else
			{
				MRPT_LOG_INFO(
					"[rx] Reset frame (start flag). SPEEDCRUISE_SERIAL_PORT");
			}
		}

		if (nFrameBytes > 2 && nFrameBytes + nRead == lengthField)
		{
			if (buf[nFrameBytes + nRead - 1] != FRAME_END_FLAG)
			{
				is_ok = false;
				MRPT_LOG_INFO("[rx] Reset frame (end flag)");
			}
		}

		MRPT_TODO("Checksum");

		if (is_ok)
		{
			nFrameBytes += nRead;
		}
		else
		{
			nFrameBytes = 0;  // No es cabecera de trama correcta
			buf[1] = buf[2] = 0;
		}
	}

	// Frame received
	lengthField = buf[2] + 5;
	rxFrame.resize(lengthField);
	::memcpy(&rxFrame[0], &buf[0], lengthField);

#ifdef DEBUG_TRACES
	std::string s;
	s += mrpt::format("RX frame (%u bytes): ", (unsigned int)lengthField);
	for (size_t i = 0; i < lengthField; i++)
		s += mrpt::format("%02X ", rxFrame[i]);
	MRPT_LOG_INFO_FMT("%s", s.c_str());
#endif

	// All OK
	return true;
}

bool VehicleControllerLowLevel::IsConnected() const
{
	return m_serial_Steer.isOpen() || m_serial_SpeedCruise.isOpen();
}

bool VehicleControllerLowLevel::CMD_Decimation_configuration(
	const TFrameCMD_VERBOSITY_CONTROL_payload_t& Decimation_config,
	CSerialPort& m_serial)
{
	TFrameCMD_VERBOSITY_CONTROL cmd;
	cmd.payload = Decimation_config;
	cmd.calc_and_update_checksum();

	for (int retry = 0; retry < 3; retry++)
	{
		if (SendFrameAndWaitAnswer(
				reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd), m_serial))
			return true;
	}
	MRPT_LOG_ERROR("CMD_Decimation_configuration(): Missed ACK!");
	return false;
}
