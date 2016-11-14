/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include "CSteerControllerLowLevel.h"
#include "steering-control-firmware2pc-structs.h"
#include <mrpt/system/threads.h> // for sleep()

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


CSteerControllerLowLevel::CSteerControllerLowLevel() :
	m_serial_port_name("COM11"),
	m_serial_port_baudrate(1000000),
	m_steer_report_freq(50)
{
}

CSteerControllerLowLevel::~CSteerControllerLowLevel()
{
}

bool CSteerControllerLowLevel::OnStartUp()
{
	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL);

	// Read parameters (if any) from the mission configuration file.

	//! @moos_param SERIAL_PORT (Default="COM11") The serial port to connect to.
	m_MissionReader.GetConfigurationParam(
#if defined(MRPT_OS_WINDOWS)
		"SERIAL_PORT_WIN",
#else
		"SERIAL_PORT_LIN",
#endif
		m_serial_port_name);

	//! @moos_param SERIAL_PORT_BAUDRATE (Default=1000000) The serial port baud rate
	m_MissionReader.GetConfigurationParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);

	//! @moos_param STEERPOS_STATUS_FREQ (Default=50) The desired frequency (Hz) for steering column state
	m_MissionReader.GetConfigurationParam("STEERPOS_STATUS_FREQ",m_steer_report_freq);

	// Try to connect...
	if (this->AttemptConnection())
	{
		MOOSTrace("Connection OK to steering controller.\n");

		this->SetAppFreq( std::max(500.0, 2.0*m_steer_report_freq) );

		CMD_SetReportFreq(m_steer_report_freq);
		CMD_SetPWMValue(0);

		// Send control parameters to controller?
		//MRPT_TODO("Continue!")
	}

	//! @moos_var	STEERCONTROL_AUTO_POS  Set to 0.0 to disable automatic control (=Manual steering), set to 1.0 to enable the automatic steering controller.
	//! @moos_subscribe	STEERCONTROL_AUTO_POS
	AddMOOSVariable_OpenMORA("STEERCONTROL_AUTO_POS", 0.0);

	//! @moos_var	STEERCONTROL_CLUTCH  Set to !=0.0 to enable the clutch that allows the steering motor to move the steering wheel.
	//! @moos_subscribe	STEERCONTROL_CLUTCH
	AddMOOSVariable_OpenMORA("STEERCONTROL_CLUTCH", 0.0);

	//! @moos_var	STEERCONTROL_PWM  The PWM duty cycle when in non-controller mode (STEERCONTROL_AUTO_POS=0). Range is -1.0 to +1.0 for forward/backward direction. 0 means motor stopped.
	//! @moos_subscribe	STEERCONTROL_PWM
	AddMOOSVariable_OpenMORA("STEERCONTROL_PWM", 1e-3);

	//! @moos_var	STEERCONTROL_STEERANG The desired setpoint for the steering angle (in RADIANS) when the controller is enabled (STEERCONTROL_AUTO_POS=1). 0 means straight forwards.
	//! @moos_subscribe	STEERCONTROL_STEERANG
	AddMOOSVariable_OpenMORA("STEERCONTROL_STEERANG", 1e-3);


	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CSteerControllerLowLevel::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CSteerControllerLowLevel::Iterate()
{
	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;

	if (!m_serial.isOpen())
		return false;

	std::vector<uint8_t> rxFrame;
	while (++nFrames<MAX_FRAMES_PER_ITERATE && ReceiveFrameFromController(rxFrame))
	{
		if (rxFrame.size()==sizeof(TStatusReport))
		{
			TStatusReport rep;
			memcpy(&rep,&rxFrame[0],rxFrame.size());

			//!  @moos_var STEERCONTROLSTATE_POS The current tick count (position) of the steering encoder (in ticks)
			//!  @moos_publish STEERCONTROLSTATE_POS
			m_Comms.Notify( "STEERCONTROLSTATE_POS", (double)rep.encoder_pos );

			//!  @moos_var STEERCONTROLSTATE_VEL The current steering encoder velocity (in ticks/s)
			//!  @moos_publish STEERCONTROLSTATE_VEL
			const double steerVelTicks = rep.encoder_last_incr_period!=0 ? (rep.encoder_last_incr / (rep.encoder_last_incr_period*1e-4)) : 0.0;
			m_Comms.Notify( "STEERCONTROLSTATE_VEL", steerVelTicks );

			//!  @moos_var STEERCONTROLSTATE_STEER_ANG The current Ackermann central-equivalent steering angle (in radians). 0: pointing forward; >0: clockwise (Read from the controller, not the desired setpoint!)
			//!  @moos_publish STEERCONTROLSTATE_STEER_ANG
			m_Comms.Notify( "STEERCONTROLSTATE_STEER_ANG", ackermannAngle(rep.encoder_pos) );

			//!  @moos_var STEERCONTROLSTATE_CUR_SENSE The latest voltage read by the the current sensor of the Pololu controller (in Volts)
			//!  @moos_publish STEERCONTROLSTATE_CUR_SENSE
			const double cur_sense_volts = rep.current_sense_adc * 5.0/1024.0;
			m_Comms.Notify( "STEERCONTROLSTATE_CUR_SENSE", cur_sense_volts );

			//!  @moos_var STEERCONTROLSTATE A string with "FIRMWARE_TIMESTAMP  POS_TICK VEL_TICKS"
			m_Comms.Notify( "STEERCONTROLSTATE",  mrpt::format("%f %ld %.3f %.3f", rep.timestamp*1e-4, (long)rep.encoder_pos, steerVelTicks, cur_sense_volts)  );
		}
	}

	// Process updates of: STEERCONTROL_AUTO_POS
	{
		CMOOSVariable *pVarControlAuto = GetMOOSVar_OpenMORA("STEERCONTROL_AUTO_POS");
		if(pVarControlAuto && pVarControlAuto->IsFresh())
		{
			pVarControlAuto->SetFresh(false);

			const  bool controlAutoMode = pVarControlAuto->GetDoubleVal() != 0.0;
			MOOSTrace("Setting control auto mode %s\n", controlAutoMode ? "ON":"OFF" );
			if (!CMD_EnableAutoControl(controlAutoMode)) {
				MOOSTrace("*** Error sending control auto mode!!! ***\n");
			}
		}
	}

	// Process updates of: STEERCONTROL_CLUTCH
	{
		CMOOSVariable *pVar = GetMOOSVar_OpenMORA("STEERCONTROL_CLUTCH");
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);

			const  bool val = pVar->GetDoubleVal() != 0.0;
			MOOSTrace("Setting clutch %s\n", val ? "ON":"OFF" );
			if (!CMD_SetClutch(val)) {
				MOOSTrace("*** Error sending clutch!!! ***\n");
			}
		}
	}

	// Process updates of: STEERCONTROL_PWM
	{
		CMOOSVariable *pVar = GetMOOSVar_OpenMORA("STEERCONTROL_PWM");
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);

			const double val = pVar->GetDoubleVal();
			MOOSTrace("Setting PWM %f\n", val);
			if (!CMD_SetPWMValue(val)) {
				MOOSTrace("*** Error sending PWM!!! ***\n");
			}
		}
	}


	// Process updates of: STEERCONTROL_STEERANG
	{
		CMOOSVariable *pVar = GetMOOSVar_OpenMORA("STEERCONTROL_STEERANG");
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);

			const double valAng = pVar->GetDoubleVal(); // desired angle in radians
			const int valTicks  = ackermannAngle_inverse( valAng ); // Desired setpoint, in encoder ticks

			MOOSTrace("Setting STEERANG = %f deg (=%d ticks)\n", RAD2DEG(valAng),valTicks);
			if (!CMD_SetPosControlSetPoint(valTicks)) {
				MOOSTrace("*** Error sending STEERANG!!! ***\n");
			}
		}
	}


	return true;
}

bool CSteerControllerLowLevel::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CSteerControllerLowLevel::DoRegistrations()
{
	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CSteerControllerLowLevel::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}


 //!< Returns true if the serial comms are working
bool CSteerControllerLowLevel::IsConnected() const
{
	return m_serial.isOpen();
}

bool CSteerControllerLowLevel::AttemptConnection()
{
	if (m_serial.isOpen()) return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100,0,10,0,50);

		return true;
	}
	catch (exception &e)
	{
		cerr << "[CSteerControllerLowLevel::AttemptConnection] COMMS error: " << e.what() << endl;
		return false;
	}
}


/** Sends a binary packet, in the expected format  (returns false on COMMS error) */
bool CSteerControllerLowLevel::WriteBinaryFrame( const uint8_t *data, uint16_t len)
{
	if (!AttemptConnection()) return false;

	std::vector<uint8_t> buf;

	buf.resize(len+1+2+1);
	buf[0] = STEERCONTROL_COMMS_FRAME_START_FLAG;
	buf[1] = len & 0xFF;
	buf[2] = (len>>8) & 0xFF;
	memcpy( &buf[3], data, len);
	buf[len+3] = STEERCONTROL_COMMS_FRAME_END_FLAG;

	try
	{
		m_serial.WriteBuffer(&buf[0],buf.size());
		return true;
	}
	catch (std::exception &)
	{
		return false;
	}
}

bool CSteerControllerLowLevel::ReceiveFrameFromController(std::vector<uint8_t> &rxFrame)
{
	rxFrame.clear();
	size_t	nFrameBytes = 0;
	std::vector<uint8_t> buf;
	buf.resize(0x10000);
	buf[0] = buf[1] = 0;

	size_t	lengthField;

	//                                   START_FLAG    LENGTH_FIELD       DATA                     END_FLAG
	while ( nFrameBytes < (lengthField=(    1        +     2       +   (buf[1] | (buf[2] << 8))) +     1      )  )
	{
		if (lengthField>1000)
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[1]=buf[2]=0;
			cout << "[rx] Reset frame (invalid len)\n";
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
			cerr << "[CSteerControllerLowLevel::ReceiveFrameFromController] Comms error: " << e.what() << endl;
			return false;
		}

		if ( !nRead && !nFrameBytes )
		{
			//cout << "[rx] No frame (buffer empty)\n";
			return false;
		}

		if (nRead<nBytesToRead)
			mrpt::system::sleep(1);

		// Lectura OK:
		// Check start flag:
		bool is_ok = true;

		if (!nFrameBytes && buf[0]!= STEERCONTROL_COMMS_FRAME_START_FLAG )
		{
			is_ok = false;
			//cout << "[rx] Reset frame (start flag)\n";
		}

		if (nFrameBytes>2 && nFrameBytes+nRead==lengthField)
		{
			if (buf[nFrameBytes+nRead-1]!=STEERCONTROL_COMMS_FRAME_END_FLAG)
			{
				is_ok= false;
				//cout << "[rx] Reset frame (end flag)\n";
			}
			//else { cout << "[rx] Frame OK\n"; }
		}

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
	// ----------------------------
	// | START | LEN | DATA | END |
	// ----------------------------
	lengthField= (buf[1] | (buf[2] << 8));
	rxFrame.resize(lengthField);
	memcpy( &rxFrame[0], &buf[3], lengthField);

	//MRPT_TODO("Add CRC???")

	// All OK
	return true;
}


//!< Sets the automatic/manual control in the steering controller board. Return false on COMMS error
bool CSteerControllerLowLevel::CMD_EnableAutoControl(bool enable)
{
	TCmdSetAutoMode cmd;
	cmd.enable_pos_control = enable ? 1:0;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the clutch
bool CSteerControllerLowLevel::CMD_SetClutch(bool enable)
{
	TCmdSetClutch cmd;
	cmd.relay_state = enable ? 1:0;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the report frequency
bool CSteerControllerLowLevel::CMD_SetReportFreq(double freq)
{
	const TFirmwareParams params;
	TCmdSetReportDecimation cmd;
	double dec = (freq>0) ? (1.0/(params.READ_ENCODERS_PERIOD*1e-4*freq)) : 10;
	cmd.report_decimation = dec;

	MOOSTrace("[CSteerControllerLowLevel] Setting report freq to %.02f Hz (dec=%f)\n", freq,dec);

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the PWM value (-1023,1023)
bool CSteerControllerLowLevel::CMD_SetPWMValue(double pwm_duty_cycle)
{
	TCmdSetPWMValue cmd;
	cmd.pwm_value = pwm_duty_cycle*1023;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the position setpoint
bool CSteerControllerLowLevel::CMD_SetPosControlSetPoint(int pos_ticks)
{
	TCmdSetPosControlSetPoint cmd;
	cmd.setpoint_ticks = pos_ticks;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}
