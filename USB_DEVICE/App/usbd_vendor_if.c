/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_Vendor_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_vendor_if.h"

/* USER CODE BEGIN INCLUDE */
// #include "uart_comms.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_Vendor_IF
  * @{
  */

/** @defgroup USBD_Vendor_IF_Private_TypesDefinitions USBD_Vendor_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Private_Defines USBD_Vendor_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

// volatile uint8_t read_to_idle_enabled = 0;
// volatile uint16_t rxIndex = 0;
// volatile uint16_t rxMaxSize = 0;
// uint8_t* pRX = 0;

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Private_Macros USBD_Vendor_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Private_Variables USBD_Vendor_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t VenUserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB Vendor are stored in this buffer   */
uint8_t VenUserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Exported_Variables USBD_Vendor_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_Vendor_IF_Private_FunctionPrototypes USBD_Vendor_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t Vendor_Init_HS(void);
static int8_t Vendor_DeInit_HS(void);
static int8_t Vendor_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t Vendor_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t Vendor_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_Vendor_ItfTypeDef USBD_VEN_Interface_fops_HS =
{
  Vendor_Init_HS,
  Vendor_DeInit_HS,
  Vendor_Control_HS,
  Vendor_Receive_HS,
  Vendor_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the Vendor media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Vendor_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  printf("Vendor IF Init\r\n");
  USBD_Vendor_SetTxBuffer(&hUsbDeviceHS, VenUserTxBufferHS, 0);
  USBD_Vendor_SetRxBuffer(&hUsbDeviceHS, VenUserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the Vendor media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Vendor_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the Vendor class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Vendor_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case Vendor_SEND_ENCAPSULATED_COMMAND:

    break;

  case Vendor_GET_ENCAPSULATED_RESPONSE:

    break;

  case Vendor_SET_COMM_FEATURE:

    break;

  case Vendor_GET_COMM_FEATURE:

    break;

  case Vendor_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case Vendor_SET_LINE_CODING:

    break;

  case Vendor_GET_LINE_CODING:

    break;

  case Vendor_SET_CONTROL_LINE_STATE:

    break;

  case Vendor_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over Vendor interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on Vendor interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t Vendor_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  // /* USER CODE BEGIN 11 */
  // USBD_Vendor_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  // uint16_t len = (uint16_t) *Len; // Get length
  // uint16_t tempHeadPos = rxIndex;
  // //printf("receive data\r\n");

  // if(read_to_idle_enabled == 1){
	//   // Restart timer when data is received
	//   HAL_TIM_Base_Stop_IT(&htim12);
  //   __HAL_TIM_SET_COUNTER(&htim12, 0); // Reset the timer counter

	//   if(pRX){
	// 	  for (uint32_t i = 0; i < len; i++) {
	// 		pRX[tempHeadPos] = Buf[i];
	// 	  	tempHeadPos = (uint16_t)((uint16_t)(tempHeadPos + 1) % rxMaxSize);

	// 	    if (tempHeadPos == rxIndex) {
	// 	      return USBD_FAIL;
	// 	    }
	// 	  }
	//   }
	//   rxIndex = tempHeadPos;
	//   //printf("start idle timer\r\n");
	//   HAL_TIM_Base_Start_IT(&htim12);
//  }

  USBD_Vendor_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over Vendor interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t Vendor_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hVendor->TxState != 0){
    return USBD_BUSY;
  }
  USBD_Vendor_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_Vendor_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  Vendor_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Vendor_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  // Vendor_handle_TxCpltCallback();
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void Vendor_FlushRxBuffer_HS() {

}

void Vendor_ReceiveToIdle(uint8_t* Buf, uint16_t max_size)
{
  //   rxIndex = 0;
  //   rxMaxSize = max_size;
  //   pRX = Buf;
	// read_to_idle_enabled = 1;
}

extern void Vendor_handle_RxCpltCallback(uint16_t len);
void Vendor_Idle_Timer_Handler()
{
	// read_to_idle_enabled = 0;
	// HAL_TIM_Base_Stop_IT(&htim12);

	// if(pRX){
	// 	// printf("Vendor_handle_RxCpltCallback %d \r\n", rxIndex);
	// 	Vendor_handle_RxCpltCallback(rxIndex);
	// }else{
	// 	printf("RX EMPTY\r\n");
	// 	Vendor_handle_RxCpltCallback(0);
	// }

  //   rxMaxSize = 0;
  //   pRX = 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
