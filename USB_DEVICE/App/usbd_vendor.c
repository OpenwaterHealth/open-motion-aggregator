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
    NULL  /* Get Configuration Descriptor */
};

/**
  * @brief  USBD_Vendor_Init
  *         Initialize the vendor-specific interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
USBD_StatusTypeDef USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    /* Open the Isochronous IN Endpoint */
    USBD_LL_OpenEP(pdev, USB_VENDOR_IN_EP, USBD_EP_TYPE_ISOC, USB_VENDOR_MAX_PACKET_SIZE);

    /* Prepare the first packet */
    USBD_Vendor_SendData(pdev, Vendor_Buffer, USB_VENDOR_MAX_PACKET_SIZE);

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
