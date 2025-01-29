/*
 * if_commands.h
 *
 *  Created on: Jan 29, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_IF_COMMANDS_H_
#define INC_IF_COMMANDS_H_

#include "common.h"
#include "utils.h"

void process_basic_command(UartPacket *uartResp, UartPacket* cmd);

#endif /* INC_IF_COMMANDS_H_ */
