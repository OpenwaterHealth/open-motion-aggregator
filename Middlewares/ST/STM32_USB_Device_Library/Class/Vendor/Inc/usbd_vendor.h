/**
  ******************************************************************************
  * @file    usbd_Vendor.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_Vendor.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_Vendor_H
#define __USB_Vendor_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_Vendor
  * @brief This file is the Header file for usbd_Vendor.c
  * @{
  */


/** @defgroup usbd_Vendor_Exported_Defines
  * @{
  */
#ifndef Vendor_IN_EP
#define Vendor_IN_EP                                   0x81U  /* EP1 for data IN */
#endif /* Vendor_IN_EP */
#ifndef Vendor_OUT_EP
#define Vendor_OUT_EP                                  0x01U  /* EP1 for data OUT */
#endif /* Vendor_OUT_EP */
#ifndef Vendor_CMD_EP
#define Vendor_CMD_EP                                  0x82U  /* EP2 for Vendor commands */
#endif /* Vendor_CMD_EP  */

#ifndef Vendor_HS_BINTERVAL
#define Vendor_HS_BINTERVAL                            0x10U
#endif /* Vendor_HS_BINTERVAL */

#ifndef Vendor_FS_BINTERVAL
#define Vendor_FS_BINTERVAL                            0x10U
#endif /* Vendor_FS_BINTERVAL */

/* Vendor Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define Vendor_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define Vendor_DATA_FS_MAX_PACKET_SIZE                 64U  /* Endpoint IN & OUT Packet size */
#define Vendor_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */

#define USB_Vendor_CONFIG_DESC_SIZ                     25U
#define Vendor_DATA_HS_IN_PACKET_SIZE                  Vendor_DATA_HS_MAX_PACKET_SIZE
#define Vendor_DATA_HS_OUT_PACKET_SIZE                 Vendor_DATA_HS_MAX_PACKET_SIZE

#define Vendor_DATA_FS_IN_PACKET_SIZE                  Vendor_DATA_FS_MAX_PACKET_SIZE
#define Vendor_DATA_FS_OUT_PACKET_SIZE                 Vendor_DATA_FS_MAX_PACKET_SIZE

#define Vendor_REQ_MAX_DATA_SIZE                       0x7U
/*---------------------------------------------------------------------*/
/*  Vendor definitions                                                    */
/*---------------------------------------------------------------------*/
#define Vendor_SEND_ENCAPSULATED_COMMAND               0x00U
#define Vendor_GET_ENCAPSULATED_RESPONSE               0x01U
#define Vendor_SET_COMM_FEATURE                        0x02U
#define Vendor_GET_COMM_FEATURE                        0x03U
#define Vendor_CLEAR_COMM_FEATURE                      0x04U
#define Vendor_SET_LINE_CODING                         0x20U
#define Vendor_GET_LINE_CODING                         0x21U
#define Vendor_SET_CONTROL_LINE_STATE                  0x22U
#define Vendor_SEND_BREAK                              0x23U

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} USBD_Vendor_LineCodingTypeDef;

typedef struct _USBD_Vendor_Itf
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} USBD_Vendor_ItfTypeDef;


typedef struct
{
  uint32_t data[Vendor_DATA_HS_MAX_PACKET_SIZE / 4U];      /* Force 32-bit alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_Vendor_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_Vendor;
#define USBD_Vendor_CLASS &USBD_Vendor
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_Vendor_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_Vendor_ItfTypeDef *fops);

#ifdef USE_USBD_COMPOSITE
uint8_t USBD_Vendor_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff,
                             uint32_t length, uint8_t ClassId);
uint8_t USBD_Vendor_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t ClassId);
#else
uint8_t USBD_Vendor_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff,
                             uint32_t length);
uint8_t USBD_Vendor_TransmitPacket(USBD_HandleTypeDef *pdev);
#endif /* USE_USBD_COMPOSITE */
uint8_t USBD_Vendor_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t USBD_Vendor_ReceivePacket(USBD_HandleTypeDef *pdev);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_Vendor_H */
/**
  * @}
  */

/**
  * @}
  */

