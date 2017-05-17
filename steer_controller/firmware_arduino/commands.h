/*****************************************************************************************
 FILE: commands.h

Jose Luis Blanco Claraco (C) 2005-2014
Universidad de Malaga
Universidad de Almeria
****************************************************************************************/
#ifndef CLARAQUINO_FIRM_COMMANDS_H
#define CLARAQUINO_FIRM_COMMANDS_H

/** Process a command from the PC and send any required response back.
  * \return false on error, unknown command, etc.
  */
bool process_command(const uint8_t *cmd, const uint16_t cmd_len);

#endif
