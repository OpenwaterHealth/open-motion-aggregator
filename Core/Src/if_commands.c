/*
 * if_commands.c
 *
 *  Created on: Jan 29, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"

#include <stdio.h>
#include <string.h>

extern uint8_t FIRMWARE_VERSION_DATA[3];
static uint32_t id_words[3] = {0};

void process_basic_command(UartPacket *uartResp, UartPacket* cmd)
{
	switch (cmd->command)
	{
	case OW_CMD_NOP:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_PING:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_PONG;
		break;
	case OW_CMD_PONG:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_PING;
		break;
	case OW_CMD_VERSION:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_HWID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd->id;
		uartResp->packet_type = cmd->packet_type;
		uartResp->command = cmd->command;
		uartResp->data_len = cmd->data_len;
		uartResp->data = cmd->data;
		break;
	case OW_CMD_TOGGLE_LED:
		uartResp->id = cmd->id;
		uartResp->command = OW_CMD_TOGGLE_LED;
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

static void print_uart_packet(const UartPacket* packet) __attribute__((unused));
static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

