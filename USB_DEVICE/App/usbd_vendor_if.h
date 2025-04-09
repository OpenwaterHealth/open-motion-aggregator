/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_Vendor_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_Vendor_if.c file.
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

#ifndef __USBD_Vendor_IF_H__
#define __USBD_Vendor_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_vendor.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_Vendor_IF USBD_Vendor_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_Vendor_IF_Exported_Defines USBD_Vendor_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over Vendor */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE BEGIN EXPORTED_DEFINES */

 void Vendor_FlushRxBuffer_HS();
 void Vendor_ReceiveToIdle(uint8_t* Buf, uint16_t max_size);
 void Vendor_Idle_Timer_Handler();

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Exported_Types USBD_Vendor_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Exported_Macros USBD_Vendor_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Exported_Variables USBD_Vendor_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** Vendor Interface callback. */
extern USBD_Vendor_ItfTypeDef USBD_VEN_Interface_fops_HS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Exported_FunctionsPrototype USBD_Vendor_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t Vendor_Transmit_HS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_Vendor_IF_H__ */

