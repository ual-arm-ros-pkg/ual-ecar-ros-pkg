﻿/*
 * process_command.cpp
 *
 * Created: 04/10/2017 12:15:44
 *  Author: Francisco José Mañas
 */

#include "batterycharge_declarations.h"
#include <libclaraquino/gpio.h>
#include <libclaraquino/uart.h>
#include <libclaraquino/millis_timer.h>

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
	UART::Write(rx,sizeof(rx));
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

			const uint8_t pin_vals = data[0];
			DDRD |= 0xCF;
			PORTD = (PORTD & 0x30) | (pin_vals << 2);

			// send answer back:
			send_simple_opcode_frame(RESP_SET_OPTO);
		}
		break;
		case OP_START_CONT_ADC:
		{
			if (datalen!=sizeof(TFrameCMD_ADC_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			TFrameCMD_ADC_start_payload_t adc_req;
			memcpy(&adc_req,data, sizeof(adc_req));

			adc_process_start_cmd(adc_req);
			// send answer back:
			send_simple_opcode_frame(RESP_START_CONT_ADC);
		}
		break;
		case OP_STOP_CONT_ADC:
		{
			if (datalen!=sizeof(TFrameCMD_ADC_stop_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

			adc_process_stop_cmd();

			// send answer back:
			send_simple_opcode_frame(RESP_STOP_CONT_ADC);
		}
		break;

		case OP_VERBOSITY_CONTROL:
		{
			if (datalen!=sizeof(TFrameCMD_VERBOSITY_CONTROL_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
			TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control;
			memcpy(&verbosity_control,data, sizeof(verbosity_control));
			setVerbosityControl(verbosity_control);
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
