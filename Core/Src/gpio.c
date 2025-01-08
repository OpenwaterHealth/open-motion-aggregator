/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PC9   ------> RCC_MCO_2
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ERROR_LED_Pin|MUX_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FS_OUT_EN_GPIO_Port, FS_OUT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ERROR_LED_Pin MUX_RESET_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin|MUX_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_8_Pin PC12 PC4 IMU_INT_Pin */
  GPIO_InitStruct.Pin = GPIO0_8_Pin|GPIO_PIN_12|GPIO_PIN_4|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_7_Pin GPIO0_5_Pin PB8 */
  GPIO_InitStruct.Pin = GPIO0_7_Pin|GPIO0_5_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_RESET_Pin */
  GPIO_InitStruct.Pin = USB_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_6_Pin CRESET_8_Pin CRESET_7_Pin PE0
                           PE10 PE14 GPIO0_2_Pin GPIO0_3_Pin
                           CRESET_1_Pin CRESET_3_Pin CRESET_2_Pin CRESET_4_Pin */
  GPIO_InitStruct.Pin = GPIO0_6_Pin|CRESET_8_Pin|CRESET_7_Pin|GPIO_PIN_0
                          |GPIO_PIN_10|GPIO_PIN_14|GPIO0_2_Pin|GPIO0_3_Pin
                          |CRESET_1_Pin|CRESET_3_Pin|CRESET_2_Pin|CRESET_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CRESET_6_Pin PD2 PD3 CRESET_5_Pin
                           PD4 PD0 PD15 PD11
                           GPIO0_4_Pin */
  GPIO_InitStruct.Pin = CRESET_6_Pin|GPIO_PIN_2|GPIO_PIN_3|CRESET_5_Pin
                          |GPIO_PIN_4|GPIO_PIN_0|GPIO_PIN_15|GPIO_PIN_11
                          |GPIO0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 GPIO0_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO0_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FSIN_EN_Pin */
  GPIO_InitStruct.Pin = FSIN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSIN_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FS_OUT_EN_Pin */
  GPIO_InitStruct.Pin = FS_OUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FS_OUT_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
