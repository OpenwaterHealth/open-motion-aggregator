/*
 * camera_manager.h
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_CAMERA_MANAGER_H_
#define INC_CAMERA_MANAGER_H_
#include "main.h"

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
	bool 			streaming_enabled;
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

#define CAMERA_COUNT	8
#define HISTOGRAM_DATA_SIZE	4100
#define WIDTH 1920
#define HEIGHT 1280
#define HISTOGRAM_BINS 1024
#define HISTO_TEST_PATTERN 1

void init_camera_sensors(void);
CameraDevice* get_active_cam(void);
CameraDevice* set_active_camera(int id);
CameraDevice* get_camera_byID(int id);
void switch_frame_buffer(void);
uint8_t* get_active_frame_buffer(void);
uint8_t* get_inactive_frame_buffer(void);
void fill_frame_buffers(void);
int toggle_camera_stream(uint8_t cam_id);
void SendHistogramData(void);

#endif /* INC_CAMERA_MANAGER_H_ */
