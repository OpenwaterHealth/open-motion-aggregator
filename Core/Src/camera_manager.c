/*
 * camera_manager.c
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"

#include <stdio.h>

CameraDevice cam_array[CAMERA_COUNT];	// array of all the cameras

static int _active_cam_idx = 0;

volatile uint8_t frame_buffer[2][CAMERA_COUNT * HISTOGRAM_DATA_SIZE]; // Double buffer
static uint8_t _active_buffer = 0; // Index of the buffer currently being written to


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
	init_camera(&cam_array[2]);

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
