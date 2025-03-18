#ifndef __USBD_VENDOR_H
#define __USBD_VENDOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/* Define Vendor-Specific Interface Class */
#define USB_VENDOR_CLASS    0xFF
#define USB_VENDOR_SUBCLASS 0x00
#define USB_VENDOR_PROTOCOL 0x00

/* Endpoint Address */
#define USB_VENDOR_IN_EP  0x83  /* Isochronous IN Endpoint */
#define USB_VENDOR_MAX_PACKET_SIZE 1024 /* Max packet size for HS Isochronous */
#define USB_VENDOR_INTERVAL 1 /* 1ms interval (for HS) */

/* USB Vendor Class Structure */
extern USBD_ClassTypeDef USBD_VendorClassDriver;

/* Function Prototypes */
USBD_StatusTypeDef USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_Vendor_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_Vendor_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_Vendor_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
void USBD_Vendor_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_VENDOR_H */
