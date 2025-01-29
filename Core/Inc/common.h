/*
 * common.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define MAX_BITSTREAM_SIZE 200 * 1024
#define COMMAND_MAX_SIZE 2048

#define SPI_PACKET_LENGTH 4096
#define USART_PACKET_LENGTH 4100

typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} USTX_ProtocolTypes;


typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_FPGA = 0xE6,
	OW_CAMERA = 0xE7,
	OW_I2C_PASSTHRU = 0xE9,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} UartPacketTypes;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} UstxErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_PONG = 0x01,
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_I2C_BROADCAST = 0x06,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F,
	OW_TOGGLE_CAMERA_STREAM = 0x07
} UstxGlobalCommands;

typedef enum {
	OW_FPGA_SCAN = 0x10,
	OW_FPGA_ON = 0x11,
	OW_FPGA_OFF = 0x12,
	OW_FPGA_ACTIVATE = 0x13,
	OW_FPGA_ID = 0x14,
	OW_FPGA_ENTER_SRAM_PROG = 0x15,
	OW_FPGA_EXIT_SRAM_PROG = 0x16,
	OW_FPGA_ERASE_SRAM = 0x17,
	OW_FPGA_PROG_SRAM = 0x18,
	OW_FPGA_BITSTREAM = 0x19,
	OW_FPGA_USERCODE = 0x1D,
	OW_FPGA_STATUS = 0x1E,
	OW_FPGA_RESET = 0x1F,
	OW_FPGA_SOFT_RESET = 0x1A,
	OW_HISTO = 0x1B,
} MotionFPGACommands;

typedef enum {
	OW_CAMERA_SCAN = 0x20,
	OW_CAMERA_ON = 0x21,
	OW_CAMERA_OFF = 0x22,
	OW_CAMERA_READ_TEMP = 0x24,
	OW_CAMERA_FSIN_ON = 0x26,
	OW_CAMERA_FSIN_OFF = 0x27,
	OW_CAMERA_SWITCH = 0x28,
	OW_CAMERA_SET_CONFIG = 0x29,
	OW_CAMERA_FSIN_EX_ENABLE = 0x2A,
	OW_CAMERA_FSIN_EX_DISABLE = 0x2B,
	OW_CAMERA_STATUS = 0x2E,
	OW_CAMERA_RESET = 0x2F,

} MotionCAMERACommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

typedef struct {
	uint16_t id;
	GPIO_TypeDef * 	cresetb_port;
	uint16_t  		cresetb_pin;
	GPIO_TypeDef *	gpio0_port;
	uint16_t  		gpio0_pin;
	I2C_HandleTypeDef * pI2c;
	bool 			useUsart; // use usart over spi
	bool 			useDma;
	SPI_HandleTypeDef * pSpi;
	USART_HandleTypeDef * pUart;
	uint16_t 		i2c_target;
	uint8_t 		gain;
	uint8_t 		exposure;
	uint8_t *pRecieveHistoBuffer;

} CameraDevice;

typedef struct {
	uint8_t cam0_buffer[USART_PACKET_LENGTH];
	uint8_t cam1_buffer[SPI_PACKET_LENGTH];
	uint8_t cam2_buffer[USART_PACKET_LENGTH];
	uint8_t cam3_buffer[USART_PACKET_LENGTH];
	uint8_t cam4_buffer[USART_PACKET_LENGTH];
	uint8_t cam5_buffer[SPI_PACKET_LENGTH];
	uint8_t cam6_buffer[SPI_PACKET_LENGTH];
	uint8_t cam7_buffer[SPI_PACKET_LENGTH];

} ScanPacket;


#endif /* INC_COMMON_H_ */
