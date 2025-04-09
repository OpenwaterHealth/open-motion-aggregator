/**
  ******************************************************************************
  * @file    usbd_Vendor.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB Vendor Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as Vendor Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *
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
  *  @verbatim
  *
  *          ===================================================================
  *                                Vendor Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as Vendor device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *            This driver doesn't implement the following aspects of the specification
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_vendor.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_Vendor
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_Vendor_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_Vendor_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_Vendor_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_Vendor_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Vendor_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Vendor_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_Vendor_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_Vendor_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_Vendor_EP0_RxReady(USBD_HandleTypeDef *pdev);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_Vendor_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Vendor_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_Vendor_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_Vendor_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_Vendor_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */

/** @defgroup USBD_Vendor_Private_Variables
  * @{
  */


/* Vendor interface class callbacks structure */
USBD_ClassTypeDef  USBD_Vendor =
{
  USBD_Vendor_Init,
  USBD_Vendor_DeInit,
  USBD_Vendor_Setup,
  NULL,                 /* EP0_TxSent */
  USBD_Vendor_EP0_RxReady,
  USBD_Vendor_DataIn,
  USBD_Vendor_DataOut,
  NULL,
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_Vendor_GetHSCfgDesc,
  USBD_Vendor_GetFSCfgDesc,
  USBD_Vendor_GetOtherSpeedCfgDesc,
  USBD_Vendor_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB Vendor device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_Vendor_CfgDesc[USB_Vendor_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                       /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
  USB_Vendor_CONFIG_DESC_SIZ,                    /* wTotalLength */
  0x00,
  0x01,                                       /* bNumInterfaces: 1 interfaces */
  0x01,                                       /* bConfigurationValue: Configuration value */
  0x00,                                       /* iConfiguration: Index of string descriptor
                                                 describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                       /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                       /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                             /* MaxPower (mA) */

  /*---------------------------------------------------------------------------*/

  /* Interface Descriptor */
  0x09,                                       /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,                                       /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x01,                                       /* bNumEndpoints: One endpoint used */
  0xFF,                                       /* bInterfaceClass: Vendor Specific Interface */
  0x00,                                       /* bInterfaceSubClass: Abstract Control Model */
  0x00,                                       /* bInterfaceProtocol: Common AT commands */
  0x00,                                       /* iInterface */

  // /* Header Functional Descriptor */
  // 0x05,                                       /* bLength: Endpoint Descriptor size */
  // 0x24,                                       /* bDescriptorType: CS_INTERFACE */
  // 0x00,                                       /* bDescriptorSubtype: Header Func Desc */
  // 0x10,                                       /* bVendorDC: spec release number */
  // 0x01,

  // /* Call Management Functional Descriptor */
  // 0x05,                                       /* bFunctionLength */
  // 0x24,                                       /* bDescriptorType: CS_INTERFACE */
  // 0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
  // 0x00,                                       /* bmCapabilities: D0+D1 */
  // 0x01,                                       /* bDataInterface */

  // /* ACM Functional Descriptor */
  // 0x04,                                       /* bFunctionLength */
  // 0x24,                                       /* bDescriptorType: CS_INTERFACE */
  // 0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
  // 0x02,                                       /* bmCapabilities */

  // /* Union Functional Descriptor */
  // 0x05,                                       /* bFunctionLength */
  // 0x24,                                       /* bDescriptorType: CS_INTERFACE */
  // 0x06,                                       /* bDescriptorSubtype: Union func desc */
  // 0x00,                                       /* bMasterInterface: Communication class interface */
  // 0x01,                                       /* bSlaveInterface0: Data Class Interface */

  // /* Endpoint 2 Descriptor */
  // 0x07,                                       /* bLength: Endpoint Descriptor size */
  // USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  // Vendor_CMD_EP,                                 /* bEndpointAddress */
  // 0x03,                                       /* bmAttributes: Interrupt */
  // LOBYTE(Vendor_CMD_PACKET_SIZE),                /* wMaxPacketSize */
  // HIBYTE(Vendor_CMD_PACKET_SIZE),
  // Vendor_FS_BINTERVAL,                           /* bInterval */
  /*---------------------------------------------------------------------------*/

  // /* Endpoint Data class interface descriptor */
  // 0x09,                                       /* bLength: Endpoint Descriptor size */
  // USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
  // 0x01,                                       /* bInterfaceNumber: Number of Interface */
  // 0x00,                                       /* bAlternateSetting: Alternate setting */
  // 0x02,                                       /* bNumEndpoints: Two endpoints used */
  // 0x0A,                                       /* bInterfaceClass: Vendor */
  // 0x00,                                       /* bInterfaceSubClass */
  // 0x00,                                       /* bInterfaceProtocol */
  // 0x00,                                       /* iInterface */

  // /* Endpoint OUT Descriptor */
  // 0x07,                                       /* bLength: Endpoint Descriptor size */
  // USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  // Vendor_OUT_EP,                                 /* bEndpointAddress */
  // 0x02,                                       /* bmAttributes: Bulk */
  // LOBYTE(Vendor_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  // HIBYTE(Vendor_DATA_FS_MAX_PACKET_SIZE),
  // 0x00,                                       /* bInterval */

  /* Endpoint IN Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  Vendor_IN_EP,                                  /* bEndpointAddress */
  0x01,                                       /* bmAttributes: Isochronous */
  LOBYTE(Vendor_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(Vendor_DATA_FS_MAX_PACKET_SIZE),
  0x01                                        /* bInterval */
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t VendorInEpAdd = Vendor_IN_EP;
static uint8_t VendorOutEpAdd = Vendor_OUT_EP;
static uint8_t VendorCmdEpAdd = Vendor_CMD_EP;

/**
  * @}
  */

/** @defgroup USBD_Vendor_Private_Functions
  * @{
  */

/**
  * @brief  USBD_Vendor_Init
  *         Initialize the Vendor interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_Vendor_HandleTypeDef *hVendor;

  hVendor = (USBD_Vendor_HandleTypeDef *)USBD_malloc(sizeof(USBD_Vendor_HandleTypeDef));

  if (hVendor == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  (void)USBD_memset(hVendor, 0, sizeof(USBD_Vendor_HandleTypeDef));

  pdev->pClassDataCmsit[pdev->classId] = (void *)hVendor;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  VendorInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  VendorOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  VendorCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, VendorInEpAdd, USBD_EP_TYPE_BULK,
                         Vendor_DATA_HS_IN_PACKET_SIZE);

    pdev->ep_in[VendorInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, VendorOutEpAdd, USBD_EP_TYPE_BULK,
                         Vendor_DATA_HS_OUT_PACKET_SIZE);

    pdev->ep_out[VendorOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for Vendor CMD Endpoint */
    pdev->ep_in[VendorCmdEpAdd & 0xFU].bInterval = Vendor_HS_BINTERVAL;
  }
  else
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, VendorInEpAdd, USBD_EP_TYPE_BULK,
                         Vendor_DATA_FS_IN_PACKET_SIZE);

    pdev->ep_in[VendorInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, VendorOutEpAdd, USBD_EP_TYPE_BULK,
                         Vendor_DATA_FS_OUT_PACKET_SIZE);

    pdev->ep_out[VendorOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for CMD Endpoint */
    pdev->ep_in[VendorCmdEpAdd & 0xFU].bInterval = Vendor_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  (void)USBD_LL_OpenEP(pdev, VendorCmdEpAdd, USBD_EP_TYPE_INTR, Vendor_CMD_PACKET_SIZE);
  pdev->ep_in[VendorCmdEpAdd & 0xFU].is_used = 1U;

  hVendor->RxBuffer = NULL;

  /* Init  physical Interface components */
  ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

  /* Init Xfer states */
  hVendor->TxState = 0U;
  hVendor->RxState = 0U;

  if (hVendor->RxBuffer == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, VendorOutEpAdd, hVendor->RxBuffer,
                                 Vendor_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, VendorOutEpAdd, hVendor->RxBuffer,
                                 Vendor_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_Vendor_Init
  *         DeInitialize the Vendor layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_Vendor_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);


#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this Vendor class instance */
  VendorInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  VendorOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  VendorCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, VendorInEpAdd);
  pdev->ep_in[VendorInEpAdd & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, VendorOutEpAdd);
  pdev->ep_out[VendorOutEpAdd & 0xFU].is_used = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, VendorCmdEpAdd);
  pdev->ep_in[VendorCmdEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[VendorCmdEpAdd & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_Vendor_Setup
  *         Handle the Vendor specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_Vendor_Setup(USBD_HandleTypeDef *pdev,
                              USBD_SetupReqTypedef *req)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint16_t len;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  if (hVendor == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U)
        {
          ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                           (uint8_t *)hVendor->data,
                                                                           req->wLength);

          len = MIN(Vendor_REQ_MAX_DATA_SIZE, req->wLength);
          (void)USBD_CtlSendData(pdev, (uint8_t *)hVendor->data, len);
        }
        else
        {
          hVendor->CmdOpCode = req->bRequest;
          hVendor->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hVendor->data, hVendor->CmdLength);
        }
      }
      else
      {
        ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                         (uint8_t *)req, 0U);
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, &ifalt, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state != USBD_STATE_CONFIGURED)
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_Vendor_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_Vendor_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_Vendor_HandleTypeDef *hVendor;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if ((pdev->ep_in[epnum & 0xFU].total_length > 0U) &&
      ((pdev->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
  {
    /* Update the packet total length */
    pdev->ep_in[epnum & 0xFU].total_length = 0U;

    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    hVendor->TxState = 0U;

    if (((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt != NULL)
    {
      ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hVendor->TxBuffer, &hVendor->TxLength, epnum);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_Vendor_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_Vendor_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  hVendor->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */

  ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hVendor->RxBuffer, &hVendor->RxLength);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_Vendor_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_Vendor_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hVendor == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((pdev->pUserData[pdev->classId] != NULL) && (hVendor->CmdOpCode != 0xFFU))
  {
    ((USBD_Vendor_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hVendor->CmdOpCode,
                                                                     (uint8_t *)hVendor->data,
                                                                     (uint16_t)hVendor->CmdLength);
    hVendor->CmdOpCode = 0xFFU;
  }

  return (uint8_t)USBD_OK;
}
#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_Vendor_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_Vendor_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = Vendor_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = Vendor_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = Vendor_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_Vendor_CfgDesc);
  return USBD_Vendor_CfgDesc;
}

/**
  * @brief  USBD_Vendor_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_Vendor_GetHSCfgDesc(uint16_t *length)
{
//  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_CMD_EP);
//  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_IN_EP);

//  if (pEpCmdDesc != NULL)
//  {
//    pEpCmdDesc->bInterval = Vendor_HS_BINTERVAL;
//  }
//
//  if (pEpOutDesc != NULL)
//  {
//    pEpOutDesc->wMaxPacketSize = Vendor_DATA_HS_MAX_PACKET_SIZE;
//  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = Vendor_DATA_HS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_Vendor_CfgDesc);
  return USBD_Vendor_CfgDesc;
}

/**
  * @brief  USBD_Vendor_GetOtherSpeedCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_Vendor_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_Vendor_CfgDesc, Vendor_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = Vendor_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = Vendor_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = Vendor_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_Vendor_CfgDesc);
  return USBD_Vendor_CfgDesc;
}

/**
  * @brief  USBD_Vendor_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_Vendor_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_Vendor_DeviceQualifierDesc);

  return USBD_Vendor_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_Vendor_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_Vendor_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_Vendor_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_Vendor_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @param  length: length of data to be sent
  * @param  ClassId: The Class ID
  * @retval status
  */
#ifdef USE_USBD_COMPOSITE
uint8_t USBD_Vendor_SetTxBuffer(USBD_HandleTypeDef *pdev,
                             uint8_t *pbuff, uint32_t length, uint8_t ClassId)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_Vendor_SetTxBuffer(USBD_HandleTypeDef *pdev,
                             uint8_t *pbuff, uint32_t length)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif /* USE_USBD_COMPOSITE */

  if (hVendor == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hVendor->TxBuffer = pbuff;
  hVendor->TxLength = length;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_Vendor_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_Vendor_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hVendor == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hVendor->RxBuffer = pbuff;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_Vendor_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @param  ClassId: The Class ID
  * @retval status
  */
#ifdef USE_USBD_COMPOSITE
uint8_t USBD_Vendor_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t ClassId)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_Vendor_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif  /* USE_USBD_COMPOSITE */

  USBD_StatusTypeDef ret = USBD_BUSY;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  VendorInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, ClassId);
#endif  /* USE_USBD_COMPOSITE */

  if (hVendor == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hVendor->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hVendor->TxState = 1U;

    /* Update the packet total length */
    pdev->ep_in[VendorInEpAdd & 0xFU].total_length = hVendor->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, VendorInEpAdd, hVendor->TxBuffer, hVendor->TxLength);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_Vendor_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_Vendor_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_Vendor_HandleTypeDef *hVendor = (USBD_Vendor_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  VendorOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, VendorOutEpAdd, hVendor->RxBuffer,
                                 Vendor_DATA_HS_OUT_PACKET_SIZE);
  }
  else
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, VendorOutEpAdd, hVendor->RxBuffer,
                                 Vendor_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t)USBD_OK;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

