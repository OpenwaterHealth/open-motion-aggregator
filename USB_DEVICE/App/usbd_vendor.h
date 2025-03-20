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

#define USB_VEN_CONFIG_DESC_SIZ                     (9U+ 16U)
#define USB_INTERFACE_COUNT 1
#define USB_VENDOR_EP_ADDR    0x83  // Endpoint 1, IN, Isochronous
#define USB_VENDOR_EP_SIZE    1024  // Max 1024 bytes for HS
#define USB_VENDOR_EP_INTERVAL 4    // 4 = 1ms for HS (unit = 125µs)
#define USB_VENDOR_INTERFACE  0
#define USB_INTERFACE_COUNT  1  // CDC (2) + Vendor (1)
typedef struct
{
  uint32_t data[512U / 4U];      /* Force 32-bit alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_VEN_HandleTypeDef;

/* Endpoint Address */
#define USB_VENDOR_IN_EP  0x83  /* Isochronous IN Endpoint */
#define USB_VENDOR_MAX_PACKET_SIZE 1024 /* Max packet size for HS Isochronous */
#define USB_VENDOR_INTERVAL 1 /* 1ms interval (for HS) */

/* USB Vendor Class Structure */
extern USBD_ClassTypeDef USBD_VendorClassDriver;

typedef struct {
    int8_t (*Init)        (void);   // Initialize Vendor Class
    int8_t (*DeInit)      (void);   // Deinitialize Vendor Class
    int8_t (*DataIn)      (USBD_HandleTypeDef *pdev, uint8_t epnum);  // Host requests IN data
    int8_t (*DataOut)     (USBD_HandleTypeDef *pdev, uint8_t epnum);  // Host sends OUT data
} USBD_VEN_ItfTypeDef;

/* Function Prototypes */
USBD_StatusTypeDef USBD_Vendor_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_Vendor_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_Vendor_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_Vendor_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t *USBD_VEN_GetDeviceQualifierDescriptor(uint16_t *length);
static uint8_t *USBD_VEN_GetOtherSpeedCfgDesc(uint16_t *length);

static uint8_t *USBD_VEN_GetHSCfgDesc(uint16_t *length);

void USBD_Vendor_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t length);

uint8_t USBD_VEN_RegisterInterface(USBD_HandleTypeDef *pdev,
                                   USBD_VEN_ItfTypeDef *fops);
#ifdef __cplusplus
}
#endif

#endif /* __USBD_VENDOR_H */
