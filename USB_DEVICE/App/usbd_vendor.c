#include "usbd_vendor.h"
#include "usbd_ctlreq.h"

static uint8_t Vendor_Buffer[USB_VENDOR_MAX_PACKET_SIZE];

/* USB Vendor Class Callbacks */
USBD_ClassTypeDef USBD_VendorClassDriver = 
{
    USBD_Vendor_Init,
    USBD_Vendor_DeInit,
    USBD_Vendor_Setup,
    NULL, /* EP0_TxReady */
    NULL, /* EP0_RxReady */
    USBD_Vendor_DataIn,
    NULL, /* DataOut (not used) */
    NULL, /* SOF */
    NULL, /* IsoINIncomplete */
    NULL, /* IsoOUTIncomplete */
    USBD_VEN_GetHSCfgDesc  /* Get Configuration Descriptor */
};

//uint8_t USBD_VEN_RegisterInterface(USBD_HandleTypeDef *pdev,
//                                   USBD_CDC_ItfTypeDef *fops)
//{
//  if (fops == NULL)
//  {
//    return (uint8_t)USBD_FAIL;
//  }
//
//  pdev->pUserData[pdev->classId] = fops;
//
//  return (uint8_t)USBD_OK;
//}
void InitDummyData(void) {
    for (uint16_t i = 0; i < USB_VENDOR_MAX_PACKET_SIZE; i++) {
    	Vendor_Buffer[i] = (i & 0xFF);  // Example: 0x00, 0x01, ..., 0xFF repeating
    }
}
/**
  * @brief  USBD_Vendor_Init
  *         Initialize the vendor-specific interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
USBD_StatusTypeDef USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    InitDummyData(); // Initialize the buffer with dummy data
    /* Open the Isochronous IN Endpoint */
    USBD_LL_OpenEP(pdev, USB_VENDOR_IN_EP, USBD_EP_TYPE_ISOC, USB_VENDOR_MAX_PACKET_SIZE);

    /* Prepare the first packet */
    USBD_Vendor_SendData(pdev, Vendor_Buffer, USB_VENDOR_MAX_PACKET_SIZE);
    printf("Vendor device initialized");
    return USBD_OK;
}

/**
  * @brief  USBD_Vendor_DeInit
  *         DeInitialize the vendor-specific interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
USBD_StatusTypeDef USBD_Vendor_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    /* Close the Isochronous IN Endpoint */
    USBD_LL_CloseEP(pdev, USB_VENDOR_IN_EP);
    return USBD_OK;
}

/**
  * @brief  USBD_Vendor_Setup
  *         Handle USB control requests for vendor-specific interface
  * @param  pdev: device instance
  * @param  req: USB setup request
  * @retval status
  */
USBD_StatusTypeDef USBD_Vendor_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS:
            /* Handle vendor-specific class requests here */
            break;
        
        default:
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
    }
    return USBD_OK;
}

/**
  * @brief  USBD_Vendor_DataIn
  *         Data sent over Isochronous IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
USBD_StatusTypeDef USBD_Vendor_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    /* Refill buffer for next transfer */
    USBD_Vendor_SendData(pdev, Vendor_Buffer, USB_VENDOR_MAX_PACKET_SIZE);
    printf("Vendor Data IN: %d bytes sent\n", USB_VENDOR_MAX_PACKET_SIZE);
    return USBD_OK;
}

/**
  * @brief  USBD_Vendor_SendData
  *         Sends data over the Isochronous IN endpoint
  * @param  pdev: device instance
  * @param  data: pointer to data buffer
  * @param  length: data length
  * @retval None
  */
void USBD_Vendor_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t length)
{
    /* Ensure length does not exceed max packet size */
    length = (length > USB_VENDOR_MAX_PACKET_SIZE) ? USB_VENDOR_MAX_PACKET_SIZE : length;
    
    /* Transmit data via the USB stack */
    USBD_LL_Transmit(pdev, USB_VENDOR_IN_EP, data, length);
}


__ALIGN_BEGIN static uint8_t USBD_VEN_CfgDesc[USB_VEN_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                       /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
  USB_VEN_CONFIG_DESC_SIZ,                    /* wTotalLength */
  0x00,
  USB_INTERFACE_COUNT,                        /* bNumInterfaces: 2 interfaces */
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
  /* Vendor-Specific Interface Descriptor (Interface 2) */
  0x09,                                       /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
  USB_VENDOR_INTERFACE,                       /* bInterfaceNumber: Interface 2 */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x01,                                       /* bNumEndpoints: One endpoint used */
  0xFF,                                       /* bInterfaceClass: Vendor-Specific */
  0x00,                                       /* bInterfaceSubClass */
  0x00,                                       /* bInterfaceProtocol */
  0x00,                                       /* iInterface */

  /* Vendor-Specific Isochronous IN Endpoint Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  USB_VENDOR_EP_ADDR,                           /* bEndpointAddress: IN direction */
  0x01,                                       /* bmAttributes: Isochronous */ //TODO this is a guess! verify later, may need to be adaptive
  LOBYTE(USB_VENDOR_EP_SIZE), 
  HIBYTE(USB_VENDOR_EP_SIZE),                 /* wMaxPacketSize: 1024 bytes (for High-Speed) */
  USB_VENDOR_EP_INTERVAL                      /* bInterval: 1 (125 µs for HS) */
};

/**
  * @brief  USBD_CDC_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_VEN_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpVenDesc = USBD_GetEpDesc(USBD_VEN_CfgDesc, USB_VENDOR_EP_ADDR);

  // if (pEpVenDesc != NULL)
  // {
	//   pEpVenDesc->bInterval = USB_VENDOR_EP_INTERVAL;
  // }

  // if (pEpVenDesc != NULL)
  // {
	//   pEpVenDesc->wMaxPacketSize = USB_VENDOR_EP_SIZE;
  // }

  *length = (uint16_t)sizeof(USBD_VEN_CfgDesc);
  return USBD_VEN_CfgDesc;
}


/**
  * @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_VEN_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_VEN_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}