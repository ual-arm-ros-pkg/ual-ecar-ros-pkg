/*
 * process_command.cpp
 *
 * Created: 04/10/2017 12:15:44
 *  Author: Francisco José Mañas
 */

#include "batterycharge_declarations.h"
#include <libclaraquino/gpio.h>
#include <string.h>

struct TimeoutData
{
	unsigned long TIMEOUT_TICKS;   //!< Number of millis() ticks to timeout an output signal. Default=1000 ms

	bool BAT_any;
	unsigned long BAT_last_changed[7];  //!< Last timestamp (millis()) for each DAC channel

	TimeoutData() :
	TIMEOUT_TICKS(1000),
	BAT_any(false)
	{
		::memset(BAT_last_changed,0, sizeof(BAT_last_changed));
	}
};

TimeoutData PendingTimeouts;

void send_simple_opcode_frame(const uint8_t op)
{
	const uint8_t rx[] = { FRAME_START_FLAG, op, 0x00, 0x00, FRAME_END_FLAG };
	//Serial.write(rx,sizeof(rx));
}

void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data)
{
	switch (opcode)
	{
		case OP_NOP:
		{
			if (datalen!=sizeof(TFrameCMD_NOP_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			// No-operation: just a fake command to check if comms are alive
			return send_simple_opcode_frame(RESP_NOP);
		}
		break;

		case OP_SET_OPTO:
		{
			if (datalen!=sizeof(TFrameCMD_OPTO_output_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			const uint8_t pin_no = data[0];
			const uint8_t pin_val = data[1];
			gpio_pin_mode(pin_no, OUTPUT);
			gpio_pin_write(pin_no, pin_val);

			// send answer back:
			send_simple_opcode_frame(RESP_SET_OPTO);
		}
		break;
		case OP_GET_OPTO:
		{
			if (datalen!=sizeof(TFrameCMD_OPTO_read_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			const uint8_t pin_no = data[0];
			pinMode(pin_no, INPUT);
			const uint8_t val = digitalRead(pin_no);

			// send answer back:
			const uint8_t rx[] = { FRAME_START_FLAG, RESP_GET_OPTO, 0x01, pin_no, val, 0x00 +pin_no+ val/*checksum*/, FRAME_END_FLAG };
			Serial.write(rx,sizeof(rx));
		}
		break;

		case OP_START_BAT:
		{
			if (datalen!=sizeof(TFrameCMD_BATTERY_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			TFrameCMD_ENCODERS_start_payload_t bat_req;
			memcpy(&bat_req,data, sizeof(bat_req));

			init_encoders(bat_req);

			// send answer back:
			send_simple_opcode_frame(RESP_START_BAT);
		}
		break;

		case OP_STOP_BAT:
		{
			TFrameCMD_BATTERY_start_payload_t cmd_empty;
			init_bettery(cmd_empty);
			BATTERY_active=false;

			// send answer back:
			send_simple_opcode_frame(RESP_STOP_BAT);
		}
		break;

		default:
		{
			// Error:
			send_simple_opcode_frame(RESP_UNKNOWN_OPCODE);
		}
		break;
	};
}

void process_timeouts()
{
	TimeoutData &pt = PendingTimeouts; // shortcut
	const unsigned long tnow = millis();

	if (pt.BAT_any)
	{
		pt.BAT_any=false; // if no timeout is set, don't waste time in the next time we are called.
		for (uint8_t i=0;i<sizeof(pt.BAT_last_changed)/sizeof(pt.BAT_last_changed[0]);i++)
		{
			if (pt.BAT_last_changed[i]!=0)
			{
				pt.BAT_any=true;
				if (tnow - pt.BAT_last_changed[i] > pt.TIMEOUT_TICKS)
				{
					// Watchdog timer event!
					pt.BAT_last_changed[i]=0; // reset this one
					//mod_dac_max5500_update_single_DAC(i,0);
				}

			}
		}
	}
}
