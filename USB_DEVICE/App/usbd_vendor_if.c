#include "usbd_vendor_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

static int8_t VEN_Init_HS(void);
static int8_t VEN_DeInit_HS(void);
static int8_t USBD_Vendor_DataIn_HS(USBD_HandleTypeDef *pdev, uint8_t epnum) ;
static int8_t USBD_Vendor_DataOut_HS(USBD_HandleTypeDef *pdev, uint8_t epnum);

//uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

USBD_VEN_ItfTypeDef USBD_Interface_fops_HS =
{
  VEN_Init_HS,
  VEN_DeInit_HS,
  USBD_Vendor_DataIn_HS,
  USBD_Vendor_DataOut_HS,
};

static int8_t VEN_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
//   USBD_VEN_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
//   USBD_VEN_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  printf("HS USB Vendor device initialized\r\n");
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the VEN media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VEN_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

static int8_t USBD_Vendor_DataIn_HS(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    printf("USB Vendor Data IN Requested on EP 0x%02X\r\n", epnum);
    return USBD_OK;
}

static int8_t USBD_Vendor_DataOut_HS(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    printf("USB Vendor Data OUT Received on EP 0x%02X\r\n", epnum);
    return USBD_OK;
}