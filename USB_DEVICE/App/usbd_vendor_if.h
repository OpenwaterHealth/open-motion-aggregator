#ifndef __USBD_VENDOR_IF_H
#define __USBD_VENDOR_IF_H

#include "usbd_vendor.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

// this is the sender func void CDC_FlushRxBuffer_HS();
extern USBD_VEN_ItfTypeDef USBD_Interface_fops_HS;

#endif /* __USB_VEN_H */
