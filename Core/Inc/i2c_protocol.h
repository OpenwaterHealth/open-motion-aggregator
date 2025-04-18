/*
 * i2c_protocol.h
 *
 *  Created on: Mar 30, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_PROTOCOL_H_
#define INC_I2C_PROTOCOL_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_STATUS_SIZE 8
#define I2C_BUFFER_SIZE 264

typedef struct {
	uint8_t pkt_len;
	uint16_t id;
	uint8_t cmd;
	uint8_t reserved;
	uint8_t data_len;
	const uint8_t* pData;
	uint16_t crc;
} I2C_TX_Packet;

typedef struct {
	uint16_t id;	    // commands should never be 0, 0 is always an asyc
	uint8_t cmd;    	//
	uint8_t status; 	//
	uint8_t reserved;
	uint8_t data_len;
	uint16_t crc;
} I2C_STATUS_Packet;

void i2c_tx_packet_print(const I2C_TX_Packet* packet);
void i2c_status_packet_print(const I2C_STATUS_Packet* packet);

bool i2c_packet_fromBuffer(const uint8_t* buffer, I2C_TX_Packet* pTX);
size_t i2c_packet_toBuffer(I2C_TX_Packet* pTX, uint8_t* buffer);

bool i2c_status_packet_fromBuffer(const uint8_t* buffer, I2C_STATUS_Packet* pTX);
size_t i2c_status_packet_toBuffer(I2C_STATUS_Packet* pTX, uint8_t* buffer);

#endif /* INC_I2C_PROTOCOL_H_ */
