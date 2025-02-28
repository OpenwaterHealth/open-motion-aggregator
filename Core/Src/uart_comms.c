/*
 * uart_comms.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "if_commands.h"
#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include <string.h>
#include "usbd_cdc_if.h"

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE*3];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;

void UART_INTERFACE_SendDMA(UartPacket* pResp)
{
	// while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
//	if((pResp->data_len + 12) > COMMAND_MAX_SIZE){
//		printf("Data packet too long, not sending \r\n");
//		return;
//	}
	printf("Enter\r\n");
//	memset(txBuffer, 0, sizeof(txBuffer));
	uint16_t bufferIndex = 0;

	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	if(pResp->data_len > 0)
	{
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	txBuffer[bufferIndex++] = OW_END_BYTE;

	uint8_t ret = CDC_Transmit_HS(txBuffer, bufferIndex);
	if(ret != USBD_OK){
		printf("CDC_Transmit_HS %i", ret);
	}
	// HAL_UART_Transmit_DMA(&huart1, txBuffer, bufferIndex);
	while(!tx_flag);
	printf("Exit\r\n");
}

// This is the FreeRTOS task
void comms_start_task() {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	CDC_FlushRxBuffer_HS();

	UartPacket cmd;
	UartPacket resp;
    uint16_t calculated_crc;
    rx_flag = 0;
    tx_flag = 0;
    while(1) {
    	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
		while(!rx_flag);
        int bufferIndex = 0;

        if(rxBuffer[bufferIndex++] != OW_START_BYTE) {
            // Send NACK doesn't have the correct start byte
        	resp.id = cmd.id;
            resp.data_len = 0;
            resp.packet_type = OW_NAK;
            printf("Incorrect start byte\r\n");
            goto NextDataPacket;
        }

        cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;
        cmd.packet_type = rxBuffer[bufferIndex++];
        cmd.command = rxBuffer[bufferIndex++];
        cmd.addr = rxBuffer[bufferIndex++];
        cmd.reserved = rxBuffer[bufferIndex++];

        // Extract payload length
        cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;

        // Check if data length is valid
        if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && rxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
            // Send NACK response due to no end byte
        	// data can exceed buffersize but every buffer must have a start and end packet
        	// command that will send more data than one buffer will follow with data packets to complete the request
        	resp.id = cmd.id;
        	resp.addr = 0;
        	resp.reserved = 0;
            resp.data_len = 0;
            resp.packet_type = OW_NAK;
            printf("Packet too long, NACKing");
            goto NextDataPacket;
        }

        // Extract data pointer
        cmd.data = &rxBuffer[bufferIndex];
        if (cmd.data_len > COMMAND_MAX_SIZE)
        {
        	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
        }else{
        	bufferIndex += cmd.data_len; // move pointer to end of data
        }

        // Extract received CRC
        cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
        bufferIndex+=2;

        // Calculate CRC for received data

        if (cmd.data_len > COMMAND_MAX_SIZE)
        {
        	calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
        }
        else
        {
        	calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 8);
        }

        // Check CRC
        if (cmd.crc != calculated_crc) {
            // Send NACK response due to bad CRC
        	resp.id = cmd.id;
        	resp.addr = 0;
        	resp.reserved = 0;
            resp.data_len = 0;
            resp.packet_type = OW_BAD_CRC;
            printf("Bad CRC Packet detected, length %i\r\n", cmd.data_len);
            printf("CRC in packet: 0x%04X, Calculated CRC: 0x%04X\r\n", cmd.crc, calculated_crc); // Output: 0xABCD
            goto NextDataPacket;
        }

        // Check end byte
        if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
        	resp.id = cmd.id;
            resp.data_len = 0;
        	resp.addr = 0;
        	resp.reserved = 0;
            resp.packet_type = OW_NAK;
            goto NextDataPacket;
        }

		resp = process_if_command(cmd);

NextDataPacket:
		UART_INTERFACE_SendDMA(&resp);
		memset(rxBuffer, 0, sizeof(rxBuffer));
		ptrReceive=0;
		rx_flag = 0;
    }

}

// Callback functions
void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}
