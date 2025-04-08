#include "usbd_vendor.h"
#include "usbd_ctlreq.h"

static uint8_t Vendor_Buffer[USB_VENDOR_MAX_PACKET_SIZE];
volatile uint16_t buffer_head = 0;       // Points to the next empty position
volatile uint16_t buffer_tail = 0;       // Points to the next data to send
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
    USBD_Vendor_SOF, /* SOF */
    USBD_Vendor_IsoINIncomplete, /* IsoINIncomplete */
    USBD_Vendor_IsoOUTIncomplete, /* IsoOUTIncomplete */
    USBD_VEN_GetHSCfgDesc,  /* Get Configuration Descriptor */
    NULL,
    USBD_VEN_GetOtherSpeedCfgDesc,
    USBD_VEN_GetDeviceQualifierDescriptor
};

__ALIGN_BEGIN static uint8_t USBD_VEN_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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
    printf("🚀 USBD_Vendor_Init: Configuring Isochronous Endpoint\r\n");

    // Open Isochronous IN Endpoint (Device → Host)
    printf("✅ Opened EP 0x81 as Isochronous IN (1024 bytes per packet)\r\n");

    // Mark endpoint as used
    pdev->ep_in[USB_VENDOR_EP_ADDR & 0x7F].is_used = 1;
    pdev->ep_in[USB_VENDOR_EP_ADDR & 0x7F].bInterval = 1;  // Polling every 125µs
    pdev->ep_in[USB_VENDOR_EP_ADDR & 0x7F].maxpacket = 512;
    ((USBD_VEN_ItfTypeDef *)pdev->pUserData[0])->Init();


    USBD_LL_OpenEP(pdev, USB_VENDOR_EP_ADDR, USBD_EP_TYPE_ISOC, 1024);
    USBD_LL_FlushEP(pdev, USB_VENDOR_EP_ADDR);
    
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
    USBD_LL_CloseEP(pdev, USB_VENDOR_EP_ADDR);
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
    printf("USB SETUP Request: bmRequestType=0x%02X, bRequest=0x%02X, wLength=%d\r\n",
           req->bmRequest, req->bRequest, req->wLength);

    static uint8_t response_data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};  // Test response
	  
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    
      // Request Type Class
      case USB_REQ_TYPE_CLASS :		
    			switch (req->bRequest & 0x1F) {  // filter on the target
				    case USB_REQ_RECIPIENT_INTERFACE:
              printf("USB CLASS Request received! Preparing to send response...\r\n");

              // Send data response 
              USBD_CtlSendData(pdev, response_data, sizeof(response_data));
              printf("✅ Data sent to host: 8 bytes\r\n");

              return USBD_OK;

          }
    }
    

    return USBD_FAIL;
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
    printf("Vendor Data IN: %d bytes sent\r\n", USB_VENDOR_MAX_PACKET_SIZE);
    /* Refill buffer for next transfer */
    USBD_Vendor_SendData(pdev, Vendor_Buffer, USB_VENDOR_MAX_PACKET_SIZE);
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
  printf("USBD_Vendor_SendData");
    /* Ensure length does not exceed max packet size */
    length = (length > USB_VENDOR_MAX_PACKET_SIZE) ? USB_VENDOR_MAX_PACKET_SIZE : length;
    
    /* Transmit data via the USB stack */
    USBD_LL_Transmit(pdev, USB_VENDOR_EP_ADDR, data, length);
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

  if (pEpVenDesc != NULL)
  {
	   pEpVenDesc->bInterval = USB_VENDOR_EP_INTERVAL;
	   pEpVenDesc->wMaxPacketSize = USB_VENDOR_EP_SIZE;
  }

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
  printf("ClassID: %d\r\n", pdev->classId);
  return (uint8_t)USBD_OK;
}

uint8_t *USBD_VEN_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_VEN_DeviceQualifierDesc);

  return USBD_VEN_DeviceQualifierDesc;
}

static uint8_t *USBD_VEN_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_VEN_CfgDesc);
  return USBD_VEN_CfgDesc;
}
USBD_StatusTypeDef USBD_Vendor_SOF(USBD_HandleTypeDef  *pdev){
  printf("USB Vendor SOF\r\n");
  USBD_VEN_HandleTypeDef *hVEN = (USBD_VEN_HandleTypeDef *) pdev->pClassDataCmsit[pdev->classId];

  if (hVEN->TxState == 0)
  {
    (void)USBD_LL_Transmit(pdev, USB_VENDOR_EP_ADDR, hVEN->TxBuffer, hVEN->TxLength);
  }

  return (uint8_t)USBD_OK;
} /* SOF */
USBD_StatusTypeDef USBD_Vendor_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) /* IsoINIncomplete */
{
  printf("USB Vendor Isochronous IN Incomplete: %d\r\n", epnum);
    // USBD_VEN_HandleTypeDef *hvendor = (USBD_VEN_HandleTypeDef*) pdev->pClassData;

    // if (hvendor == NULL) {
    //     printf("ISO IN Incomplete: Class Data is NULL\n");
    //     return USBD_FAIL;
    // }

    // // Debug output
    // printf("ISO IN Incomplete: Host missed a packet on EP 0x%02X\n", epnum);

    // // Refill the buffer and resend data if available
    // uint16_t available = (buffer_head >= buffer_tail) ?
    //                      (buffer_head - buffer_tail) :
    //                      (USB_VENDOR_MAX_PACKET_SIZE - buffer_tail + buffer_head);

    // if (available >= hvendor->IsoPacketSize) {
    //     printf("ISO IN Incomplete: Retrying %d bytes\n", hvendor->IsoPacketSize);

    //     for (uint16_t i = 0; i < hvendor->IsoPacketSize; i++) {
    //         hvendor->TxBuffer[i] = Vendor_Buffer[buffer_tail];
    //         buffer_tail = (buffer_tail + 1) % USB_VENDOR_MAX_PACKET_SIZE;
    //     }

    //     // Attempt to resend the packet
    //     return USBD_LL_Transmit(pdev, USB_VENDOR_EP_ADDR, hvendor->TxBuffer, hvendor->IsoPacketSize);
    // } else {
    //     printf("ISO IN Incomplete: No data available, skipping retry\n");
    // }
    printf("ISO IN Incomplete: No data available, skipping retry\r\n");
    
    return USBD_OK;
}
USBD_StatusTypeDef USBD_Vendor_IsoOUTIncomplete(USBD_HandleTypeDef *hpcd, uint8_t epnum) /* IsoOUTIncomplete */
{
  printf("USB Vendor Isochronous OUT Incomplete: %d\r\n", epnum);
  return (uint8_t) USBD_OK;
}
void Vendor_EnqueueData(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
    	Vendor_Buffer[buffer_head] = data[i];
        buffer_head = (buffer_head + 1) % USB_VENDOR_MAX_PACKET_SIZE;

        // Prevent buffer overflow (overwrite old data if full)
        if (buffer_head == buffer_tail) {
            buffer_tail = (buffer_tail + 1) % USB_VENDOR_MAX_PACKET_SIZE;
        }
    }
}
