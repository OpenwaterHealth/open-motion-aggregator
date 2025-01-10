/*
 * 0X02C1B.c
 *
 *  Created on: Oct 15, 2024
 *      Author: GeorgeVigelette
 */
#include "0X02C1B.h"
#include "X02C1B_Sensor_Config.h"
#include <stdio.h>

#define I2C_TIMEOUT 1000 // Set an appropriate timeout for I2C transactions

static int X02C1B_write(I2C_HandleTypeDef * pI2c, uint16_t reg, uint8_t val)
{
    uint8_t data[3] = { reg >> 8, reg & 0xff, val };

    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
        printf("===> ERROR: I2C Not in ready state\r\n");
        return -1;
    }

    if (HAL_I2C_Master_Transmit(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), data, 3, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Transmission failed\r\n");
        Error_Handler();
    }

    return 0;
}

static int X02C1B_read(I2C_HandleTypeDef * pI2c, uint16_t reg, uint8_t *val)
{
    uint8_t buf[2] = { reg >> 8, reg & 0xff };

    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
        printf("===> ERROR: I2C Not in ready state\r\n");
        return -1;
    }

    if (HAL_I2C_Master_Transmit(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), buf, 2, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Write failed\r\n");
        Error_Handler();
    }

    if (HAL_I2C_Master_Receive(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), val, 1, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Read failed\r\n");
        Error_Handler();
    }

    return 0;
}

static int X02C1B_write_array(I2C_HandleTypeDef * pI2c, const struct regval_list *regs, int array_size)
{
    for (int i = 0; i < array_size; i++) {
        int ret = X02C1B_write(pI2c, regs[i].addr, regs[i].data);
        if (ret < 0) {
            printf("Failed to write register 0x%04X\r\n", regs[i].addr);
            return ret;
        }
    }
    return 0;
}

int X02C1B_configure_sensor(CameraDevice *cam) {

    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x00);  // Stream off register address and value
    if (ret < 0) {
        printf("Failed to stop streaming\r\n");
        return ret;
    }
    ret = X02C1B_write(cam->pI2c, 0x0107, 0x01);  // undocumented
	if (ret < 0) {
		printf("Failed to stop streaming\r\n");
		return ret;
	}

	HAL_Delay(100);
    ret = X02C1B_write_array(cam->pI2c, X02C1B_SENSOR_CONFIG, ARRAY_SIZE(X02C1B_SENSOR_CONFIG));
    if (ret < 0) {
        printf("Sensor configuration failed\r\n");
        return ret;
    }
    printf("Sensor successfully configured\r\n");

	HAL_Delay(100);
    return 0;
}

int X02C1B_soft_reset(CameraDevice *cam) {
    int ret = X02C1B_write(cam->pI2c, 0x0103, 0x01);  // Stream on register address and value
    if (ret < 0) {
        printf("Failed to reset device\r\n");
        return ret;
    }
    printf("Streaming started\r\n");
    return 0;
}

int X02C1B_stream_on(CameraDevice *cam) {
    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x01);  // Stream on register address and value
    if (ret < 0) {
        printf("Failed to start streaming\r\n");
        return ret;
    }
    printf("Streaming started\r\n");
    return 0;
}

int X02C1B_stream_off(CameraDevice *cam) {
    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x00);  // Stream off register address and value
    if (ret < 0) {
        printf("Failed to stop streaming\r\n");
        return ret;
    }
    printf("Streaming stopped\r\n");
    return 0;
}

int X02C1B_detect(CameraDevice *cam)
{
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(cam->pI2c, X02C1B_ADDRESS << 1, 2, I2C_TIMEOUT);
    if (status != HAL_OK) {
        printf("Device Not Ready\r\n");
        return -1;
    }

    int ret = X02C1B_write(cam->pI2c, X02C1B_SW_RESET, 0x01);
    if (ret < 0) {
        printf("Failed to write SW_RESET\r\n");
        return ret;
    }

    uint8_t read;
    ret = X02C1B_read(cam->pI2c, X02C1B_EC_A_REG03, &read);
    if (ret < 0) {
        printf("Failed to read EC_A_REG03, got %02X\r\n", read);
        return ret;
    }

    printf("Register Value: 0x%02X\r\n", read);
    return 0;
}

int X02C1B_fsin_on()
{
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	   HAL_GPIO_WritePin(FS_OUT_EN_GPIO_Port, FS_OUT_EN_Pin, GPIO_PIN_RESET); //D12
	  printf("COMPLETED\r\n");

      return 0;
}
int X02C1B_fsin_off()
{
    while (HAL_GPIO_ReadPin(FSIN_GPIO_Port, FSIN_Pin) == GPIO_PIN_SET)
    {
        osDelay(1); // wait until the frame sync is done to keep a partial cycle from spitting out
    }
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(FS_OUT_EN_GPIO_Port, FS_OUT_EN_Pin, GPIO_PIN_SET); //D12
    printf("COMPLETED\r\n");
    return 0;
}

float X02C1B_read_temp(CameraDevice *cam)
{
	// Read temperature bytes
    uint8_t upper_byte;
    int ret = X02C1B_read(cam->pI2c, X02C1B_TEMP_UPPER, &upper_byte);
    if (ret < 0) {
        printf("Failed to read X02C1B_TEMP_UPPER, got %02X\r\n", upper_byte);
        return ret;
    }
    uint8_t lower_byte;
    ret = X02C1B_read(cam->pI2c, X02C1B_TEMP_LOWER, &lower_byte);
    if (ret < 0) {
        printf("Failed to read X02C1B_TEMP_LOWER, got %02X\r\n", lower_byte);
        return ret;
    }

    uint16_t bytes = (upper_byte << 8) + lower_byte;
    float temperature;
    if(bytes < 0xC000)  //temperature is positive
    	temperature = upper_byte + (0.001f * lower_byte);
	else
		temperature = (0xC0 - upper_byte) + (0.001f * lower_byte);

    printf("Camera %i Temperature: %f degC (0x%X)\r\n",cam->id+1,temperature,bytes);

    return temperature;
}
