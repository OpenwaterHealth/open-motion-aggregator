/*
 * if_commands.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "jsmn.h"
#include "crosslink.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#include "0X02C1B.h"

#include <stdio.h>
#include <string.h>

extern uint8_t FIRMWARE_VERSION_DATA[3];
static uint32_t id_words[3] = {0};
static float temp;

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_PING:
		uartResp->command = OW_CMD_PING;
		break;
	case OW_CMD_VERSION:
		uartResp->command = OW_CMD_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->command = OW_CMD_HWID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->command = OW_CMD_ECHO;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		printf("Toggle LED\r\n");
		uartResp->command = OW_CMD_TOGGLE_LED;
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		break;
	case OW_CMD_I2C_BROADCAST:
		printf("Broadcasting I2C on all channels\r\n");
		TCA9548A_SelectBroadcast(pCam->pI2c, 0x70);
		break;
	case OW_TOGGLE_CAMERA_STREAM:
		uartResp->command = OW_TOGGLE_CAMERA_STREAM;
		if (cmd.data_len == 1)
		{
			uint8_t cam_id = cmd.data[0];
			if (cam_id < 8)
			{
				toggle_camera_stream(cam_id);
			    uartResp->packet_type = OW_ACK;
			}
			else
			{
				uartResp->packet_type = OW_ERROR;
				printf("Invalid camera ID: %d\r\n", cam_id);
			}
		}
		else
		{
			uartResp->packet_type = OW_ERROR;
			printf("Invalid data length: %d\r\n", cmd.data_len);
		}
		break;
	case OW_CMD_RESET:
		uartResp->command = OW_CMD_RESET;
		// softreset
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

extern uint8_t bitstream_buffer[];
uint8_t* ptrBitstream;
uint32_t Bitstream_Length;

void I2C_DisableEnableReset(I2C_HandleTypeDef *hi2c)
{
    // Step 1: Disable the I2C peripheral
    __HAL_RCC_I2C1_CLK_DISABLE(); // Replace I2C1 with your I2C instance
    HAL_I2C_DeInit(hi2c);         // De-initialize the I2C to reset its state

    // Step 2: Add a small delay for safety
    HAL_Delay(10);

    // Step 3: Re-enable the I2C peripheral
    __HAL_RCC_I2C1_CLK_ENABLE(); // Re-enable the I2C clock
    HAL_I2C_Init(hi2c);          // Reinitialize the I2C

    // Optional: Verify the I2C is ready
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
        // Handle error, e.g., log a message or reset the microcontroller
        // printf("I2C Reset Failed\n");
    }
}

static void process_fpga_commands(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_FPGA_ON:
		uartResp->command = OW_FPGA_ON;
		fpga_on(pCam);
		break;
	case OW_FPGA_OFF:
		uartResp->command = OW_FPGA_OFF;
		fpga_off(pCam);
		break;
	case OW_FPGA_ACTIVATE:
		uartResp->command = OW_FPGA_ACTIVATE;
		fpga_send_activation(pCam);
		break;
	case OW_FPGA_ID:
		uartResp->command = OW_FPGA_ID;
#if 0
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
#endif
		fpga_checkid(pCam);
		break;
	case OW_FPGA_ENTER_SRAM_PROG:
		uartResp->command = OW_FPGA_ENTER_SRAM_PROG;
		fpga_enter_sram_prog_mode(pCam);
		break;
	case OW_FPGA_EXIT_SRAM_PROG:
		uartResp->command = OW_FPGA_EXIT_SRAM_PROG;
		fpga_exit_prog_mode(pCam);
		break;
	case OW_FPGA_ERASE_SRAM:
		uartResp->command = OW_FPGA_ERASE_SRAM;
		fpga_erase_sram(pCam);
		break;
	case OW_FPGA_PROG_SRAM:
		uartResp->command = OW_FPGA_PROG_SRAM;
		fpga_program_sram(pCam, true, 0, 0);
		break;
	case OW_FPGA_USERCODE:
		uartResp->command = OW_FPGA_USERCODE;
		fpga_read_usercode(pCam);
		break;
	case OW_FPGA_BITSTREAM:
		uartResp->command = OW_FPGA_BITSTREAM;
		//printf("ADDR: %d Length: %d \r\n", cmd.addr, cmd.data_len);
		if(cmd.reserved == 0){
			// printf("Received BLOCK: %d\r\n", cmd.addr);
			if(cmd.addr == 0)
			{
				uint8_t writeArr[4];
				writeArr[0] = 0x7A; writeArr[1] = 0x00; writeArr[2] = 0x00; writeArr[3] = 0x00;
				memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);
				ptrBitstream = bitstream_buffer;
				Bitstream_Length = 0;
				memcpy(ptrBitstream, writeArr, 4);  // Copy command data
				ptrBitstream += 4;
				Bitstream_Length+=4;
			}

			memcpy(ptrBitstream, cmd.data, cmd.data_len);  // Copy data
			ptrBitstream += cmd.data_len;
			Bitstream_Length+=cmd.data_len;
		}else{
			printf("Programming BITSTREAM\r\n");
			fpga_program_sram(pCam, false, bitstream_buffer, Bitstream_Length);
		}
		break;
	case OW_FPGA_STATUS:
		uartResp->command = OW_FPGA_STATUS;
		fpga_read_status(pCam);
		break;
	case OW_FPGA_RESET:
		uartResp->command = OW_FPGA_RESET;
		fpga_reset(pCam);
		HAL_Delay(1);

		I2C_DisableEnableReset(pCam->pI2c);
		break;
	case OW_FPGA_SOFT_RESET:
		uartResp->command = OW_FPGA_SOFT_RESET;
		fpga_soft_reset(pCam);
		if(pCam->useUsart){
			// method 1: clear the rxfifo
//			cam.pUart->Instance->RQR |= USART_RQR_RXFRQ;
			// method 2: diable and reenable the usart
			pCam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
			pCam->pUart->Instance->CR1 |= USART_CR1_UE;
			printf("Usart buffer reset\r\n");
		}
		break;
	case OW_FPGA_SCAN:
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}


static void process_camera_commands(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_CAMERA_SCAN:
		printf("Reading Camera %d ID\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_SCAN;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_detect(pCam)){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_ON:
		printf("Setting Camera %d Stream on\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_ON;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_stream_on(pCam)){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_SET_CONFIG:
		printf("Setting Camera %d config\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_SET_CONFIG;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_configure_sensor(pCam)<0){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_OFF:
		printf("Setting Camera %d Stream off\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_OFF;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_stream_off(pCam)<0){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_STATUS:
		printf("Camera %d status not implemented\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_STATUS;
		uartResp->packet_type = OW_RESP;
		break;
	case OW_CAMERA_RESET:
		printf("Camera %d Sensor Reset\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_RESET;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_soft_reset(pCam)<0){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_FSIN_ON:
		printf("Enabling FSIN...\r\n");
		uartResp->command = OW_CAMERA_FSIN_ON;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_fsin_on()<0){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_FSIN_OFF:
		printf("Disabling FSIN...\r\n");
		uartResp->command = OW_CAMERA_FSIN_OFF;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_fsin_off()<0){
			// error
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_SWITCH:
		uint8_t channel = cmd.data[0];
		printf("Switching to camera %d\r\n",channel+1);
        TCA9548A_SelectChannel(pCam->pI2c, 0x70, channel);
        set_active_camera(channel);
		break;
	case OW_CAMERA_READ_TEMP:
		printf("Reading Camera %d Temp\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_READ_TEMP;
		uartResp->packet_type = OW_RESP;
		temp = X02C1B_read_temp(pCam);
		if(temp<0){
			// error
			uartResp->packet_type = OW_ERROR;
	        uartResp->data_len = 0;
	        uartResp->data = NULL; // No valid data to send
		}else{
	        uartResp->data_len = sizeof(temp);
	        uartResp->data = (uint8_t *)&temp; // Point to the static temp variable
		}
		break;
	case OW_CAMERA_FSIN_EX_ENABLE:
		printf("Enabling FSIN_EXT...\r\n");
		uartResp->command = OW_CAMERA_FSIN_EX_ENABLE;
		uartResp->packet_type = OW_RESP;
		X02C1B_FSIN_EXT_enable();
		break;
	case OW_CAMERA_FSIN_EX_DISABLE:
		printf("Disabling FSIN_EXT...\r\n");
		uartResp->command = OW_CAMERA_FSIN_EX_DISABLE;
		uartResp->packet_type = OW_RESP;
		X02C1B_FSIN_EXT_disable();
		break;

	default:
		uartResp->data_len = 0;
		uartResp->command = cmd.command;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}

}

static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	// json parser
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];
    jsmnerr_t ret = jsmn_parse(&parser, (char *)cmd.data, cmd.data_len, t,
				 sizeof(t) / sizeof(t[0]), NULL);
    printf("Found %d Tokens\r\n", ret);
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
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

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	I2C_TX_Packet i2c_packet;
	CameraDevice* pCam = get_active_cam();
	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.data_len = 0;
	uartResp.data = 0;
	switch (cmd.packet_type)
	{
	case OW_JSON:
		JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	case OW_FPGA:
		process_fpga_commands(&uartResp, cmd);
		break;
	case OW_CAMERA:
		process_camera_commands(&uartResp, cmd);
		break;
	case OW_I2C_PASSTHRU:

//		print_uart_packet(&cmd);

        // printBuffer(cmd.data, 10);
		i2c_packet_fromBuffer(cmd.data, &i2c_packet);
		// i2c_tx_packet_print(&i2c_packet);

		HAL_Delay(20);

		send_buffer_to_slave(pCam->pI2c, cmd.command, cmd.data, cmd.data_len);

		break;
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}
