/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    ux_device_cdc_acm.c
 * @author  MCD Application Team
 * @brief   USBX Device CDC ACM applicative source file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "if_commands.h"
#include "utils.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

#define MAX_PACKET_DATA_SIZE 2042

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA      0x01
#define TX_NEW_TRANSMITTED_DATA   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8  8
#define VCP_WORDLENGTH9  9

/* the minimum baudrate */
#define MIN_BAUDRATE     9600

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = ".UsbxAppSection"
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION) /* ARM Compiler 5/6 */
__attribute__((section(".UsbxAppSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".UsbxAppSection")))


#endif
uint8_t tx_buffer[APP_TX_DATA_SIZE];
uint8_t rx_buffer[APP_RX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

UART_HandleTypeDef *uart_handler;
extern TX_EVENT_FLAGS_GROUP EventFlag;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding = { 115200, /* baud rate */
0x00, /* stop bits-1 */
0x00, /* parity - none */
0x08 /* nb. of bits 8 */
};

static UartPacketCallback packet_callback = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern VOID USBX_APP_UART_Init(UART_HandleTypeDef **huart);
static void USBD_CDC_VCP_Config(UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void RegisterUartPacketCallback(UartPacketCallback callback) {
	packet_callback = callback;
}
/* USER CODE END 0 */

/**
 * @brief  USBD_CDC_ACM_Activate
 *         This function is called when insertion of a CDC ACM device.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance) {
	/* USER CODE BEGIN USBD_CDC_ACM_Activate */

	/* Save the CDC instance */
	cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

	/* Configure the UART peripheral */
	/* Get default UART parameters */
	CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate =
			uart_handler->Init.BaudRate;

	/* Set the UART data type : only 8bits and 9bits are supported */
	switch (uart_handler->Init.WordLength) {
	case UART_WORDLENGTH_8B: {
		/* Set UART data bit to 8 */
		CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit =
		VCP_WORDLENGTH8;
		break;
	}

	case UART_WORDLENGTH_9B: {
		/* Set UART data bit to 9 */
		CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit =
		VCP_WORDLENGTH9;
		break;
	}

	default: {
		/* By default set UART data bit to 8 */
		CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit =
		VCP_WORDLENGTH8;
		break;
	}
	}

	/* Get UART Parity */
	CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_parity =
			uart_handler->Init.Parity;

	/* Get UART StopBits */
	CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_stop_bit =
			uart_handler->Init.StopBits;

	/* Set device class_cdc_acm with default parameters */
	if (ux_device_class_cdc_acm_ioctl(cdc_acm,
	UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
			&CDC_VCP_LineCoding) != UX_SUCCESS) {
		Error_Handler();
	}

	/* USER CODE END USBD_CDC_ACM_Activate */

	return;
}

/**
 * @brief  USBD_CDC_ACM_Deactivate
 *         This function is called when extraction of a CDC ACM device.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance) {
	/* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);

	/* Reset the cdc acm instance */
	cdc_acm = UX_NULL;

	/* DeInitialize the UART peripheral */
	HAL_UART_DeInit(uart_handler);
	/* USER CODE END USBD_CDC_ACM_Deactivate */

	return;
}

/**
 * @brief  USBD_CDC_ACM_ParameterChange
 *         This function is invoked to manage the CDC ACM class requests.
 * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
 * @retval none
 */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance) {
	/* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);

	ULONG request;
	UX_SLAVE_TRANSFER *transfer_request;
	UX_SLAVE_DEVICE *device;

	/* Get the pointer to the device */
	device = &_ux_system_slave->ux_system_slave_device;

	/* Get the pointer to the transfer request associated with the control endpoint */
	transfer_request =
			&device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

	request = *(transfer_request->ux_slave_transfer_request_setup
			+ UX_SETUP_REQUEST);

	switch (request) {
	case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING:

		/* Get the Line Coding parameters */
		if (ux_device_class_cdc_acm_ioctl(cdc_acm,
		UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
				&CDC_VCP_LineCoding) != UX_SUCCESS) {
			Error_Handler();
		}

		/* Check if baudrate < 9600) then set it to 9600 */
		if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate
				< MIN_BAUDRATE) {
			CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate =
			MIN_BAUDRATE;

			/* Set the new configuration of ComPort */
			USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
		} else {
			/* Set the new configuration of ComPort */
			USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
		}

		break;

	case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING:

		/* Set the Line Coding parameters */
		if (ux_device_class_cdc_acm_ioctl(cdc_acm,
		UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
				&CDC_VCP_LineCoding) != UX_SUCCESS) {
			Error_Handler();
		}

		break;

	case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:
	default:
		break;
	}

	/* USER CODE END USBD_CDC_ACM_ParameterChange */

	return;
}

/* USER CODE BEGIN 1 */

void uart_interface_send(UartPacket* pResp)
{
	// while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	memset(tx_buffer, 0, sizeof(tx_buffer));
	uint16_t bufferIndex = 0;

	tx_buffer[bufferIndex++] = OW_START_BYTE;
	tx_buffer[bufferIndex++] = pResp->id >> 8;
	tx_buffer[bufferIndex++] = pResp->id & 0xFF;
	tx_buffer[bufferIndex++] = pResp->packet_type;
	tx_buffer[bufferIndex++] = pResp->command;
	tx_buffer[bufferIndex++] = pResp->addr;
	tx_buffer[bufferIndex++] = pResp->reserved;
	tx_buffer[bufferIndex++] = (pResp->data_len) >> 8;
	tx_buffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	if(pResp->data_len > 0)
	{
		memcpy(&tx_buffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}
	uint16_t crc = util_crc16(&tx_buffer[1], pResp->data_len + 8);
	tx_buffer[bufferIndex++] = crc >> 8;
	tx_buffer[bufferIndex++] = crc & 0xFF;

	tx_buffer[bufferIndex++] = OW_END_BYTE;

	CDC_Transmit_HS(tx_buffer, bufferIndex);

}

static void HandleUartPacket(UartPacket *packet, UartPacket *resp) {
    /* Process the received packet */

	if(resp == NULL){
		Error_Handler();
	}

	resp->id = packet->id;
	resp->packet_type = OW_RESP;
	resp->data_len = 0;
	resp->data = 0;
	switch (packet->packet_type)
	{
	case OW_JSON:
		break;
	case OW_CMD:
		process_basic_command(resp, packet);
		break;
	case OW_FPGA:
		break;
	case OW_CAMERA:
		break;
	case OW_I2C_PASSTHRU:
		break;
	default:
		resp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}

void comms_start()
{
	printf("Registering COMMS Callback\r\n");
	RegisterUartPacketCallback(HandleUartPacket);
}


VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input) {
	ULONG actual_length;
	UX_SLAVE_DEVICE *device;
	static uint16_t rx_index = 0;
	static UartPacket current_packet;
	static UartPacket resp;
	static uint16_t calculated_crc = 0;
	static uint8_t err_flag = 0;
	UX_PARAMETER_NOT_USED(thread_input);

	device = &_ux_system_slave->ux_system_slave_device;

	memset(&current_packet, 0, sizeof(UartPacket)); // Clear the packet
	memset(&resp, 0, sizeof(UartPacket)); // Clear the packet

	while (1) {
		// Check if the device is configured
		if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED)
				&& (cdc_acm != UX_NULL)) {
			// Read data in blocking mode
			ux_device_class_cdc_acm_read(cdc_acm, (UCHAR*) rx_buffer,
					sizeof(rx_buffer), &actual_length);

			calculated_crc = 0;
			rx_index = 0;
			resp.packet_type = OW_RESP;

			if (rx_buffer[rx_index++] != OW_START_BYTE) {
				// Send NACK doesn't have the correct start byte
				err_flag = 1;
	        	resp.id = OW_UNKNOWN;
	            resp.data_len = 0;
	            resp.packet_type = OW_NAK;
	            goto NextDataPacket;
			} else {
				// Extract Packet ID
				current_packet.id = (rx_buffer[rx_index] << 8 | (rx_buffer[rx_index+1] & 0xFF ));
				rx_index+=2;

				// Extract Packet Type
				current_packet.packet_type = rx_buffer[rx_index++];

				// Extract Command
				current_packet.command = rx_buffer[rx_index++];

				// Extract Address
				current_packet.addr = rx_buffer[rx_index++];

				// Extract Reserved
				current_packet.reserved = rx_buffer[rx_index++];

				// Extract payload length
				current_packet.data_len = (rx_buffer[rx_index] << 8 | (rx_buffer[rx_index + 1] & 0xFF));
				rx_index += 2;

				// Check if data length is valid
				if (current_packet.data_len > COMMAND_MAX_SIZE - rx_index && rx_buffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
					// Send NACK response due to no end byte
					err_flag = 1;
		        	resp.id = current_packet.command;
		        	resp.addr = current_packet.addr;
		        	resp.reserved = current_packet.reserved;
		            resp.data_len = 0;
		            resp.packet_type = OW_NAK;
		            goto NextDataPacket;
				}

				if(current_packet.data_len>0)
				{
					// Extract data pointer
					current_packet.data = &rx_buffer[rx_index];
					if (current_packet.data_len > COMMAND_MAX_SIZE)
					{
						rx_index=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
					}else{
						rx_index += current_packet.data_len; // move pointer to end of data
					}
				}else{
					current_packet.data = NULL;
				}

				// Extract received CRC
				current_packet.crc = (rx_buffer[rx_index] << 8 | (rx_buffer[rx_index+1] & 0xFF ));
				rx_index+=2;

				// Calculate CRC for received data
				if (current_packet.data_len > COMMAND_MAX_SIZE)
				{
					calculated_crc = util_crc16(&rx_buffer[1], COMMAND_MAX_SIZE-3);
				}
				else
				{
					calculated_crc = util_crc16(&rx_buffer[1], current_packet.data_len + 8);
				}


				// Check CRC
				if (current_packet.crc != calculated_crc) {
					// Send NACK response due to bad CRC
					err_flag = 1;
		        	resp.id = current_packet.command;
		        	resp.addr = current_packet.addr;
		        	resp.reserved = current_packet.reserved;
		            resp.data_len = 0;
		            resp.packet_type = OW_BAD_CRC;
		            goto NextDataPacket;
				}

				// Check end byte
				if (rx_buffer[rx_index++] != OW_END_BYTE) {
					// Send NACK response due to bad End Byte
					err_flag = 1;
		        	resp.id = current_packet.command;
		        	resp.addr = current_packet.addr;
		        	resp.reserved = current_packet.reserved;
		            resp.data_len = 0;
		            resp.packet_type = OW_NAK;
		            goto NextDataPacket;
				}

				if(packet_callback != NULL && err_flag == 0)
				{
					packet_callback(&current_packet, &resp);
				}else{
					printf("There was an error\r\n");
				}


NextDataPacket:
				uart_interface_send(&resp);
				memset(rx_buffer, 0, sizeof(rx_buffer));
				memset(&current_packet, 0, sizeof(UartPacket)); // Clear the packet
				memset(&resp, 0, sizeof(UartPacket)); // Clear the packet
				err_flag = 0;
			}
		} else {
			// Sleep thread for 10ms if the device is not configured
			tx_thread_sleep(MS_TO_TICK(10));
		}
	}
}

ULONG CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
    UX_SLAVE_DEVICE *device;
    device = &_ux_system_slave->ux_system_slave_device;
    ULONG actual_length = 0;
    /* Check if device is configured */
    if((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
    	if (ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)Buf, Len, &actual_length) == UX_SUCCESS)
    	{
    		return actual_length;
    	}
    }
	return 0;
}

static VOID USBD_CDC_VCP_Config(
		UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER *CDC_VCP_LineCoding) {
	/* Check stop bit parameter */
	switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_stop_bit) {
	case 0:

		/* Set the UART Stop bit to 1 */

		break;

	case 2:

		/* Set the UART Stop bit to 2 */

		break;

	default:

		/* By default set the UART Stop bit to 1 */

		break;
	}

	/* Check parity parameter */
	switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_parity) {
	case 0:

		/* Set the UART parity bit to none */

		break;

	case 1:

		/* Set the UART parity bit to ODD */

		break;

	case 2:

		/* Set the UART parity bit to even */

		break;

	default:

		/* By default set the UART parity bit to none */

		break;
	}

	/* Set the UART data type : only 8bits and 9bits is supported */
	switch (CDC_VCP_LineCoding->ux_slave_class_cdc_acm_parameter_data_bit) {
	case 0x07:

		/* With this configuration a parity (Even or Odd) must be set */

		break;

	case 0x08:
		// Parity NONE UART_WORDLENGTH_8B else UART_WORDLENGTH_9B

		break;

	default:
		// UART_WORDLENGTH_8B

		break;
	}

}

/* USER CODE END 1 */
