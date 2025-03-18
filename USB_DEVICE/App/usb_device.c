/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
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

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_vendor.h"
#include "usbd_conf.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceHS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
uint8_t CDC_EpAdd_Inst1[3] = {CDC_IN_EP, CDC_OUT_EP, CDC_CMD_EP}; /* CDC Endpoint Adress First Instance */
uint8_t VEN_EpAdd_Inst2[1] = {USB_VENDOR_EP_ADDR}; /* CDC Endpoint Adress Second Instance */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
  {
    Error_Handler();
  }
  // if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_CDC) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  /* Register the Vendor-Specific Class */
  // if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_VendorClassDriver) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  // if (USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS) != USBD_OK)
  // {
  //   Error_Handler();
  // }
  //TODO fix the RegisterClass function to allow multiple classes

  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_CDC, CLASS_TYPE_CDC, CDC_EpAdd_Inst1) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClassComposite(&hUsbDeviceHS, &USBD_VendorClassDriver, CLASS_TYPE_NONE, VEN_EpAdd_Inst2) != USBD_OK)
  {
    Error_Handler();
  }
  /* Add CDC Interface Class */
  if (USBD_CMPSIT_SetClassID(&hUsbDeviceHS, CLASS_TYPE_CDC, CDC_CLASS_ID) != 0xFF)
  {
    USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS);
  }

  /* Add CDC Interface Class */
  if (USBD_CMPSIT_SetClassID(&hUsbDeviceHS, CLASS_TYPE_NONE, VENDOR_CLASS_ID) != 0xFF)
  {
    USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS);
  }

  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

