/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_LED_Pin GPIO_PIN_14
#define ERROR_LED_GPIO_Port GPIOC
#define GPIO0_8_Pin GPIO_PIN_13
#define GPIO0_8_GPIO_Port GPIOC
#define GPIO0_7_Pin GPIO_PIN_9
#define GPIO0_7_GPIO_Port GPIOB
#define GPIO0_5_Pin GPIO_PIN_4
#define GPIO0_5_GPIO_Port GPIOB
#define USB_RESET_Pin GPIO_PIN_15
#define USB_RESET_GPIO_Port GPIOA
#define MUX_RESET_Pin GPIO_PIN_15
#define MUX_RESET_GPIO_Port GPIOC
#define GPIO0_6_Pin GPIO_PIN_3
#define GPIO0_6_GPIO_Port GPIOE
#define CRESET_6_Pin GPIO_PIN_5
#define CRESET_6_GPIO_Port GPIOD
#define CRESET_8_Pin GPIO_PIN_4
#define CRESET_8_GPIO_Port GPIOE
#define CRESET_7_Pin GPIO_PIN_1
#define CRESET_7_GPIO_Port GPIOE
#define CRESET_5_Pin GPIO_PIN_7
#define CRESET_5_GPIO_Port GPIOD
#define GPIO0_1_Pin GPIO_PIN_0
#define GPIO0_1_GPIO_Port GPIOA
#define IMU_FSYNC_Pin GPIO_PIN_1
#define IMU_FSYNC_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_5
#define IMU_INT_GPIO_Port GPIOC
#define GPIO0_2_Pin GPIO_PIN_7
#define GPIO0_2_GPIO_Port GPIOE
#define FSIN_EN_Pin GPIO_PIN_11
#define FSIN_EN_GPIO_Port GPIOE
#define GPIO0_3_Pin GPIO_PIN_15
#define GPIO0_3_GPIO_Port GPIOE
#define GPIO0_4_Pin GPIO_PIN_14
#define GPIO0_4_GPIO_Port GPIOD
#define CRESET_1_Pin GPIO_PIN_8
#define CRESET_1_GPIO_Port GPIOE
#define CRESET_3_Pin GPIO_PIN_12
#define CRESET_3_GPIO_Port GPIOE
#define FSIN_Pin GPIO_PIN_13
#define FSIN_GPIO_Port GPIOD
#define CRESET_2_Pin GPIO_PIN_9
#define CRESET_2_GPIO_Port GPIOE
#define CRESET_4_Pin GPIO_PIN_13
#define CRESET_4_GPIO_Port GPIOE
#define FS_OUT_EN_Pin GPIO_PIN_12
#define FS_OUT_EN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim8;
extern CRC_HandleTypeDef   hcrc;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim4;

extern CameraDevice cam;
extern CameraDevice cam1;
extern CameraDevice cam2;
extern CameraDevice cam3;
extern CameraDevice cam4;
extern CameraDevice cam5;
extern CameraDevice cam6;
extern CameraDevice cam7;
extern CameraDevice cam8;
extern CameraDevice cam_array[];
extern volatile uint8_t event_bits_enabled;
//extern EventGroupHandle_t xHistoRxEventGroup;

#define DEBUG_UART huart4

#define BIT_0    ( 1 << 0 )
#define BIT_1    ( 1 << 1 )
#define BIT_2    ( 1 << 2 )
#define BIT_3    ( 1 << 3 )
#define BIT_4    ( 1 << 4 )
#define BIT_5    ( 1 << 5 )
#define BIT_6    ( 1 << 6 )
#define BIT_7    ( 1 << 7 )

#define SPI_PACKET_LENGTH 4096
#define USART_PACKET_LENGTH 4100

void vTaskWaitForAllBits(void *pvParameters);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
