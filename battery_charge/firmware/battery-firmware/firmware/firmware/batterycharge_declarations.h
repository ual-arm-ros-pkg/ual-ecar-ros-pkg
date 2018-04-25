/*
 * batterycharge_declarations.h
 *
 * Created: 04/10/2017 12:10:33
 *  Author: Francisco Jose Manas
 */ 

#pragma once

#include "batterycharge2pc-structs.h"
#include <stdint.h>

// Function prototypes:
void reset_rx_buf();
void processIncommingPkts();
void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data);
//void process_timeouts();
void send_simple_opcode_frame(const uint8_t op);
void flash_led(int ntimes, int nms);
void process_batery_init();
void processBattery();
void setVerbosityControl(TFrameCMD_VERBOSITY_CONTROL_payload_t verbosity_control);

// Global variables:
extern TFrameCMD_VERBOSITY_CONTROL_payload_t global_decimate;

