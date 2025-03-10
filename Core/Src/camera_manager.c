/*
 * camera_manager.c
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "uart_comms.h"

CameraDevice cam_array[CAMERA_COUNT];	// array of all the cameras

static int _active_cam_idx = 0;

volatile uint8_t frame_buffer[2][CAMERA_COUNT * HISTOGRAM_DATA_SIZE]; // Double buffer
static uint8_t _active_buffer = 0; // Index of the buffer currently being written to
uint8_t frame_id = 0;
volatile uint8_t event_bits_enabled = 0x00; // holds the event bits for the cameras to be enabled
extern uint8_t event_bits;
extern ScanPacket scanPacketA;
extern ScanPacket scanPacketB;

static void generate_fake_histogram(uint8_t *histogram_data) {
    // Cast the byte buffer to uint32_t pointer to store histogram data
    uint32_t *histogram = (uint32_t *)histogram_data;

    // Initialize histogram bins to zero
    memset(histogram, 0, HISTOGRAM_DATA_SIZE/4);

    // Generate random 10-bit grayscale image and compute histogram
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        uint32_t pixel_value = rand() % HISTOGRAM_BINS; // Random 10-bit value (0-1023)
        histogram[pixel_value]++;
    }
}

static void init_camera(CameraDevice *cam){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Reconfigure CRESETB Pin
	HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
	GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);

	// Reconfigure GPIO0 Pin
	HAL_GPIO_DeInit(cam->gpio0_port, cam->gpio0_pin);
	GPIO_InitStruct.Pin = cam->gpio0_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->gpio0_port, &GPIO_InitStruct);

	cam->streaming_enabled = false;
}

void init_camera_sensors() {
	int i = 0;

	//Configure, initialize, and set default camera
	cam_array[0].id = 0;
	cam_array[0].cresetb_port = CRESET_1_GPIO_Port;
	cam_array[0].cresetb_pin = CRESET_1_Pin;
	cam_array[0].gpio0_port = GPIO0_1_GPIO_Port;
	cam_array[0].gpio0_pin = GPIO0_1_Pin;
	cam_array[0].useUsart = true;
	cam_array[0].useDma = true;
	cam_array[0].pI2c = &hi2c1;
	cam_array[0].pSpi = NULL;
	cam_array[0].pUart = &husart2;
	cam_array[0].i2c_target = 0;
	cam_array[0].pRecieveHistoBuffer = NULL;

	cam_array[1].id = 1;
	cam_array[1].cresetb_port = CRESET_2_GPIO_Port;
	cam_array[1].cresetb_pin = CRESET_2_Pin;
	cam_array[1].gpio0_port = GPIO0_2_GPIO_Port;
	cam_array[1].gpio0_pin = GPIO0_2_Pin;
	cam_array[1].useUsart = false;
	cam_array[1].useDma = false;
	cam_array[1].pI2c = &hi2c1;
	cam_array[1].pSpi = &hspi6;
	cam_array[1].pUart = NULL;
	cam_array[1].i2c_target = 1;
	cam_array[1].pRecieveHistoBuffer = NULL;

	cam_array[2].id = 2;
	cam_array[2].cresetb_port = CRESET_3_GPIO_Port;
	cam_array[2].cresetb_pin = CRESET_3_Pin;
	cam_array[2].gpio0_port = GPIO0_3_GPIO_Port;
	cam_array[2].gpio0_pin = GPIO0_3_Pin;
	cam_array[2].useUsart = true;
	cam_array[2].useDma = true;
	cam_array[2].pI2c = &hi2c1;
	cam_array[2].pSpi = NULL;
	cam_array[2].pUart = &husart3;
	cam_array[2].i2c_target = 2;
	cam_array[2].pRecieveHistoBuffer = NULL;

	cam_array[3].id = 3;
	cam_array[3].cresetb_port = CRESET_4_GPIO_Port;
	cam_array[3].cresetb_pin = CRESET_4_Pin;
	cam_array[3].gpio0_port = GPIO0_4_GPIO_Port;
	cam_array[3].gpio0_pin = GPIO0_4_Pin;
	cam_array[3].useUsart = true;
	cam_array[3].useDma = true;
	cam_array[3].pI2c = &hi2c1;
	cam_array[3].pSpi = NULL;
	cam_array[3].pUart = &husart6;
	cam_array[3].i2c_target = 3;
	cam_array[3].pRecieveHistoBuffer = NULL;

	cam_array[4].id = 4;
	cam_array[4].cresetb_port = CRESET_5_GPIO_Port;
	cam_array[4].cresetb_pin = CRESET_5_Pin;
	cam_array[4].gpio0_port = GPIO0_5_GPIO_Port;
	cam_array[4].gpio0_pin = GPIO0_5_Pin;
	cam_array[4].useUsart = true;
	cam_array[4].useDma = true;
	cam_array[4].pI2c = &hi2c1;
	cam_array[4].pSpi = NULL;
	cam_array[4].pUart = &husart1;
	cam_array[4].i2c_target = 4;
	cam_array[4].pRecieveHistoBuffer = NULL;

	cam_array[5].id = 5;
	cam_array[5].cresetb_port = CRESET_6_GPIO_Port;
	cam_array[5].cresetb_pin = CRESET_6_Pin;
	cam_array[5].gpio0_port = GPIO0_6_GPIO_Port;
	cam_array[5].gpio0_pin = GPIO0_6_Pin;
	cam_array[5].useUsart = false;
	cam_array[5].useDma = true;
	cam_array[5].pI2c = &hi2c1;
	cam_array[5].pSpi = &hspi3;
	cam_array[5].pUart = NULL;
	cam_array[5].i2c_target = 5;
	cam_array[5].pRecieveHistoBuffer = NULL;

	cam_array[6].id = 6;
	cam_array[6].cresetb_port = CRESET_7_GPIO_Port;
	cam_array[6].cresetb_pin = CRESET_7_Pin;
	cam_array[6].gpio0_port = GPIO0_7_GPIO_Port;
	cam_array[6].gpio0_pin = GPIO0_7_Pin;
	cam_array[6].useUsart = false;
	cam_array[6].useDma = true;
	cam_array[6].pI2c = &hi2c1;
	cam_array[6].pSpi = &hspi2;
	cam_array[6].pUart = NULL;
	cam_array[6].i2c_target = 6;
	cam_array[6].pRecieveHistoBuffer = NULL;

	cam_array[7].id = 7;
	cam_array[7].cresetb_port = CRESET_8_GPIO_Port;
	cam_array[7].cresetb_pin = CRESET_8_Pin;
	cam_array[7].gpio0_port = GPIO0_8_GPIO_Port;
	cam_array[7].gpio0_pin = GPIO0_8_Pin;
	cam_array[7].useUsart = false;
	cam_array[7].useDma = true;
	cam_array[7].pI2c = &hi2c1;
	cam_array[7].pSpi = &hspi4;
	cam_array[7].pUart = NULL;
	cam_array[7].i2c_target = 7;
	cam_array[7].pRecieveHistoBuffer = NULL;

	for(i=0; i<CAMERA_COUNT; i++){
		cam_array[i].pRecieveHistoBuffer =(uint8_t *)&frame_buffer[_active_buffer][i * HISTOGRAM_DATA_SIZE];
		init_camera(&cam_array[i]);
	}

	event_bits = 0x00;
	event_bits_enabled = 0x00;

	scanPacketA = (ScanPacket ) { 0 };
	scanPacketB = (ScanPacket ) { 0 };
//	for (int i = 0; i < 4; i++) {
//		toggle_camera_stream(i);
//	}
	toggle_camera_stream(0);
}

CameraDevice* get_active_cam(void) {
	return &cam_array[_active_cam_idx];
}

CameraDevice* set_active_camera(int id) {
	_active_cam_idx = id;
	return &cam_array[_active_cam_idx];
}

CameraDevice* get_camera_byID(int id) {
	if(id < 0 || id > 7)
		return NULL;
	return &cam_array[id];
}

void switch_frame_buffer(void) {
    _active_buffer = 1 - _active_buffer; // Toggle between 0 and 1
    // Reassign each camera’s buffer pointer
    for (int i = 0; i < CAMERA_COUNT; i++) {
        cam_array[i].pRecieveHistoBuffer =(uint8_t *)&frame_buffer[_active_buffer][i * HISTOGRAM_DATA_SIZE];
    }
}

uint8_t* get_active_frame_buffer(void) {
    return (uint8_t*)frame_buffer[_active_buffer];
}

uint8_t* get_inactive_frame_buffer(void) {
    return (uint8_t*)frame_buffer[1 - _active_buffer];
}

void fill_frame_buffers(void) {
    for (int i = 0; i < CAMERA_COUNT; i++) {
    	generate_fake_histogram(cam_array[i].pRecieveHistoBuffer);
    }
}

void SendHistogramData(void) {

	if (event_bits == event_bits_enabled && event_bits_enabled > 0) {
		event_bits = 0x00;
		frame_id++;
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

		UartPacket telem;
		telem.id = 0; // arbitrarily deciding that all telem packets have id 0
		telem.packet_type = OW_DATA;
		telem.command = OW_HISTO;
		telem.data_len = SPI_PACKET_LENGTH;
		telem.addr = 0;

		for (int i = 0; i < 8; i++) {
			CameraDevice cam = cam_array[i];
			HAL_StatusTypeDef status;

			if (cam.streaming_enabled) {
				// Step 1. send out the packet
				// just send out each histo over the buffer
				// this is vile but if it works i'm going to be upset
				HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
				telem.data = cam_array[i].pRecieveHistoBuffer;
				telem.id = 0;
				telem.addr = i;
				comms_interface_send(&telem);
				HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

//                	Step 2 Switch the buffer
//        		    cam_array[i].pRecieveHistoBuffer = (cam_array[i].pRecieveHistoBuffer == scanPacketA.cam0_buffer) ? scanPacketB.cam0_buffer : scanPacketA.cam0_buffer;

				// Step 3 set up the next event
				if (cam.useUsart) {
					if (cam.useDma) {
						status = HAL_USART_Receive_DMA(cam.pUart,
								cam.pRecieveHistoBuffer, USART_PACKET_LENGTH);
					} else {
						status = HAL_USART_Receive_IT(cam.pUart,
								cam.pRecieveHistoBuffer, USART_PACKET_LENGTH);
					}
				} else {
					if (cam.useDma) {
						status = HAL_SPI_Receive_DMA(cam.pSpi,
								cam.pRecieveHistoBuffer, SPI_PACKET_LENGTH);
					} else {
						status = HAL_SPI_Receive_IT(cam.pSpi,
								cam.pRecieveHistoBuffer, SPI_PACKET_LENGTH);
					}
				}
				if (status != HAL_OK) {
					Error_Handler();
				}

			}
		}
	}
}


int toggle_camera_stream(uint8_t cam_id){
    // add to the event bits
    printf("Event bits before toggling: %02X\r\n", event_bits_enabled);

    event_bits_enabled ^= (1 << cam_id);
    printf("Event bits after toggling: %02X\r\n", event_bits_enabled);

    bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
    get_camera_byID(cam_id)->streaming_enabled = enabled;
    HAL_StatusTypeDef status;
    if(enabled){
        printf("Enabled camera stream %d\r\n", cam_id +1);

        // kick off the reception
        if(get_camera_byID(cam_id)->useUsart) {
            if(get_camera_byID(cam_id)->useDma)
            	status = HAL_USART_Receive_DMA(get_camera_byID(cam_id)->pUart, get_camera_byID(cam_id)->pRecieveHistoBuffer, USART_PACKET_LENGTH);
            else
            	status = HAL_USART_Receive_IT(get_camera_byID(cam_id)->pUart, get_camera_byID(cam_id)->pRecieveHistoBuffer, USART_PACKET_LENGTH);
        }
        else{
            if(get_camera_byID(cam_id)->useDma)
            	status = HAL_SPI_Receive_DMA(get_camera_byID(cam_id)->pSpi, get_camera_byID(cam_id)->pRecieveHistoBuffer, SPI_PACKET_LENGTH);
            else
            	status = HAL_SPI_Receive_IT(get_camera_byID(cam_id)->pSpi, get_camera_byID(cam_id)->pRecieveHistoBuffer, SPI_PACKET_LENGTH);
        }
    }
    else{
        printf("Disabled camera stream %d\r\n", cam_id +1);
        // disable the reception
		if(get_camera_byID(cam_id)->useUsart) {
			if(get_camera_byID(cam_id)->useDma)
				status = HAL_USART_Abort(get_camera_byID(cam_id)->pUart);
			else
				status = HAL_USART_Abort_IT(get_camera_byID(cam_id)->pUart);
		}
		else{
			if(get_camera_byID(cam_id)->useDma)
				status = HAL_SPI_Abort(get_camera_byID(cam_id)->pSpi);
			else
				status = HAL_SPI_Abort_IT(get_camera_byID(cam_id)->pSpi);
		}
    }
    return status;
}

