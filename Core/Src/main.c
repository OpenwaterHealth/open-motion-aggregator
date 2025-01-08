/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "app_threadx.h"
#include "main.h"
#include "bdma.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_comms.h"
#include "logging.h"
#include "utils.h"
#include "i2c_master.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_PACKET_LENGTH 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t FIRMWARE_VERSION_DATA[3] = {1, 0, 1};
/*
osThreadId_t comTaskHandle;
const osThreadAttr_t comTask_attributes = {
  .name = "comTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t histoTaskHandle;
const osThreadAttr_t histoTask_attributes = {
  .name = "histoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
*/
uint8_t rxBuffer[COMMAND_MAX_SIZE];
uint8_t txBuffer[COMMAND_MAX_SIZE];
__attribute__((section(".RAM_D1"))) uint8_t bitstream_buffer[MAX_BITSTREAM_SIZE];  // 160KB buffer

// Spi data buffers, 2 3 4 6
uint8_t spi2RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t spi2RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoSpi2 = spi2RxBufferA;

uint8_t spi3RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t spi3RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoSpi3 = spi3RxBufferA;

uint8_t spi4RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t spi4RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoSpi4 = spi4RxBufferA;

uint8_t spi6RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t spi6RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoSpi6 = spi6RxBufferA;


uint8_t usart1RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t usart1RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoUsart1 = usart1RxBufferA;

uint8_t usart2RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t usart2RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoUsart2 = usart2RxBufferA;

uint8_t usart3RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t usart3RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoUsart3 = usart3RxBufferA;

uint8_t usart6RxBufferA[SPI_PACKET_LENGTH] = {0};
uint8_t usart6RxBufferB[SPI_PACKET_LENGTH] = {0};
uint8_t *pRecieveHistoUsart6 = usart6RxBufferA;

CameraDevice cam;
CameraDevice cam1;
CameraDevice cam2;
CameraDevice cam3;
CameraDevice cam4;
CameraDevice cam5;
CameraDevice cam6;
CameraDevice cam7;
CameraDevice cam8;

CameraDevice cam_array[8];
//EventGroupHandle_t xHistoRxEventGroup;
//osEventFlagsId_t event_flags_id;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void COMTask(void *argument);
void init_camera(CameraDevice *cam);

static void PrintI2CSpeed(I2C_HandleTypeDef* hi2c) {
    // Assuming the timing is configured in I2C_TIMINGR register
    uint32_t timing = hi2c->Init.Timing;
    uint32_t presc = (timing >> 28) & 0xF;
    uint32_t scll = (timing >> 0) & 0xFF;
    uint32_t sclh = (timing >> 8) & 0xFF;

    // Get the I2C clock source frequency
    uint32_t i2c_clk_freq = HAL_RCC_GetPCLK1Freq();

    // Calculate SCL speed (frequency) in Hz
    uint32_t scl_freq = i2c_clk_freq / ((presc + 1) * (scll + 1 + sclh + 1));

    // Print I2C speed
    printf("I2C Speed: %ld Hz\r\n", scl_freq); // Print the I2C speed in kHz
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_Init();
  MX_USART3_Init();
  MX_USART6_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */
  init_dma_logging();

  // enable I2C MUX
  HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_RESET);

  // enable USB PHY
  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);

  printf("\033c");
  fflush(stdout);
  HAL_Delay(500);
  printf("Openwater open-MOTION Aggregator FW v%d.%d.%d\r\n\r\n",FIRMWARE_VERSION_DATA[0], FIRMWARE_VERSION_DATA[1], FIRMWARE_VERSION_DATA[2]);
  printf("CPU Clock Frequency: %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
  printf("Initializing, please wait ...\r\n");
  // enable I2C MUX
  HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_SET);

  // test i2c
  PrintI2CSpeed(&hi2c1);

  if(ICM20948_IsAlive(&hi2c1,0) == HAL_OK)
	  printf("IMU detected\r\n");
  else printf("IMU detected\r\n\n");

  for(int i = 0;i<8;i++){
	  TCA9548A_SelectChannel(&hi2c1, 0x70, i);
	  HAL_Delay(10);
	  printf("I2C Scanning bus %d\r\n",i);
	  I2C_scan(&hi2c1, NULL, 0, true);
	  HAL_Delay(10);
  }

  HAL_Delay(100);


  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);


  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);

  // TODO: move to camera thread

   //Configure, initialize, and set default camera
	cam1.id = 0;
	cam1.cresetb_port = CRESET_1_GPIO_Port;
	cam1.cresetb_pin = CRESET_1_Pin;
	cam1.gpio0_port = GPIO0_1_GPIO_Port;
	cam1.gpio0_pin = GPIO0_1_Pin;
	cam1.useUsart = true;
	cam1.pI2c = &hi2c1;
	cam1.pSpi = NULL;
	cam1.pUart = &husart2;
	cam1.i2c_target = 0;
	cam_array[0] = cam1;
	init_camera(&cam1);

	cam2.id = 1;
	cam2.cresetb_port = CRESET_2_GPIO_Port;
	cam2.cresetb_pin = CRESET_2_Pin;
	cam2.gpio0_port = GPIO0_2_GPIO_Port;
	cam2.gpio0_pin = GPIO0_2_Pin;
	cam2.useUsart = false;
	cam2.pI2c = &hi2c1;
	cam2.pSpi = &hspi6;
	cam2.pUart = NULL;
	cam2.i2c_target = 1;
	cam_array[1] = cam2;
	init_camera(&cam2);

	cam3.id = 2;
	cam3.cresetb_port = CRESET_3_GPIO_Port;
	cam3.cresetb_pin = CRESET_3_Pin;
	cam3.gpio0_port = GPIO0_3_GPIO_Port;
	cam3.gpio0_pin = GPIO0_3_Pin;
	cam3.useUsart = true;
	cam3.pI2c = &hi2c1;
	cam3.pSpi = NULL;
	cam3.pUart = &husart3;
	cam3.i2c_target = 2;
	cam_array[2] = cam3;
	init_camera(&cam3);

	cam4.id = 3;
	cam4.cresetb_port = CRESET_4_GPIO_Port;
	cam4.cresetb_pin = CRESET_4_Pin;
	cam4.gpio0_port = GPIO0_4_GPIO_Port;
	cam4.gpio0_pin = GPIO0_4_Pin;
	cam4.useUsart = true;
	cam4.pI2c = &hi2c1;
	cam4.pSpi = NULL;
	cam4.pUart = &husart6;
	cam4.i2c_target = 3;
	cam_array[3] = cam4;
	init_camera(&cam4);

	cam5.id = 4;
	cam5.cresetb_port = CRESET_5_GPIO_Port;
	cam5.cresetb_pin = CRESET_5_Pin;
	cam5.gpio0_port = GPIO0_5_GPIO_Port;
	cam5.gpio0_pin = GPIO0_5_Pin;
	cam5.useUsart = true;
	cam5.pI2c = &hi2c1;
	cam5.pSpi = NULL;
	cam5.pUart = &husart1;
	cam5.i2c_target = 4;
	cam_array[4] = cam5;
	init_camera(&cam5);

	cam6.id = 5;
	cam6.cresetb_port = CRESET_6_GPIO_Port;
	cam6.cresetb_pin = CRESET_6_Pin;
	cam6.gpio0_port = GPIO0_6_GPIO_Port;
	cam6.gpio0_pin = GPIO0_6_Pin;
	cam6.useUsart = false;
	cam6.pI2c = &hi2c1;
	cam6.pSpi = &hspi3;
	cam6.pUart = NULL;
	cam6.i2c_target = 5;
	cam_array[5] = cam6;
	init_camera(&cam6);

	cam7.id = 6;
	cam7.cresetb_port = CRESET_7_GPIO_Port;
	cam7.cresetb_pin = CRESET_7_Pin;
	cam7.gpio0_port = GPIO0_7_GPIO_Port;
	cam7.gpio0_pin = GPIO0_7_Pin;
	cam7.useUsart = false;
	cam7.pI2c = &hi2c1;
	cam7.pSpi = &hspi2;
	cam7.pUart = NULL;
	cam7.i2c_target = 6;
	cam_array[6] = cam7;
	init_camera(&cam7);

	cam8.id = 7;
	cam8.cresetb_port = CRESET_8_GPIO_Port;
	cam8.cresetb_pin = CRESET_8_Pin;
	cam8.gpio0_port = GPIO0_8_GPIO_Port;
	cam8.gpio0_pin = GPIO0_8_Pin;
	cam8.useUsart = false;
	cam8.pI2c = &hi2c1;
	cam8.pSpi = &hspi4;
	cam8.pUart = NULL;
	cam8.i2c_target = 7;
	cam_array[7] = cam8;
	init_camera(&cam8);

	cam = cam6;
    TCA9548A_SelectChannel(&hi2c1, 0x70, cam.i2c_target);

//    HAL_USART_Receive_IT(&husart1, pRecieveHistoUsart1, SPI_PACKET_LENGTH);
//    HAL_USART_Receive_IT(&husart2, pRecieveHistoUsart2, SPI_PACKET_LENGTH);
//    HAL_USART_Receive_IT(&husart3, pRecieveHistoUsart3, SPI_PACKET_LENGTH);
//    HAL_USART_Receive_IT(&husart6, pRecieveHistoUsart6, SPI_PACKET_LENGTH);

//  HAL_SPI_Receive_IT(&hspi2, pRecieveHistoSpi2, SPI_PACKET_LENGTH);
//  HAL_SPI_Receive_IT(&hspi3, pRecieveHistoSpi3, SPI_PACKET_LENGTH);
//  HAL_SPI_Receive_IT(&hspi4, pRecieveHistoSpi4, SPI_PACKET_LENGTH);
//  HAL_SPI_Receive_IT(&hspi6, pRecieveHistoSpi6, SPI_PACKET_LENGTH);



  printf("System Running\r\n");
  /* USER CODE END 2 */

  MX_ThreadX_Init();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_PLL2CLKOUT_ENABLE(RCC_PLL2_DIVP);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_5);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 6;
  PeriphClkInitStruct.PLL2.PLL2N = 120;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		logging_UART_TxCpltCallback(huart);
	}
}
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart) {
    printf("handler getting hit");

    UartPacket telem;
    telem.id = 0; // Arbitrarily deciding that all telem packets have id 0
    telem.packet_type = OW_DATA;
    telem.command = OW_HISTO;
    telem.data_len = SPI_PACKET_LENGTH; // Use appropriate packet length for USART
    telem.addr = 0;
	if (husart->Instance == USART1) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart1;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart1 = (pRecieveHistoUsart1 == usart1RxBufferA) ? usart1RxBufferB : usart1RxBufferA;
        if (HAL_USART_Receive_DMA(&husart1, pRecieveHistoUsart1, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART2) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart2;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart2 = (pRecieveHistoUsart2 == usart2RxBufferA) ? usart2RxBufferB : usart2RxBufferA;
        if (HAL_USART_Receive_DMA(&husart2, pRecieveHistoUsart2, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART3) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart3;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart3 = (pRecieveHistoUsart3 == usart3RxBufferA) ? usart3RxBufferB : usart3RxBufferA;
        if (HAL_USART_Receive_DMA(&husart3, pRecieveHistoUsart3, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART6) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart6;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart6 = (pRecieveHistoUsart6 == usart6RxBufferA) ? usart6RxBufferB : usart6RxBufferA;
        if (HAL_USART_Receive_DMA(&husart6, pRecieveHistoUsart6, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
}


// Error handling callback for USART
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart) {
//    if (husart->Instance == USART2) {  // Check if the error is for USART1 (you can replace with the correct USART instance)
        // Identify specific errors using error codes
        printf("USART Error occurred: ");

        if (husart->ErrorCode & HAL_USART_ERROR_PE) {
            printf("Parity error ");
        }
        if (husart->ErrorCode & HAL_USART_ERROR_NE) {
            printf("Noise error ");
        }
        if (husart->ErrorCode & HAL_USART_ERROR_FE) {
            printf("Framing error ");
        }
        if (husart->ErrorCode & HAL_USART_ERROR_ORE) {
            printf("Overrun error ");
        }
        if (husart->ErrorCode & HAL_USART_ERROR_DMA) {
            printf("DMA transfer error ");
        }
        printf("\r\n");

        // Reset USART and buffer state
        HAL_USART_DeInit(husart);             // Deinitialize USART
        HAL_USART_Init(husart);               // Reinitialize USART

        // Re-enable interrupt reception
//        pReceiveBuffer = (pReceiveBuffer == usartRxBufferA) ? usartRxBufferB : usartRxBufferA;

        // Re-enable USART interrupt reception for the next byte
//        if (HAL_USART_Receive_IT(husart, pReceiveBuffer, USART_PACKET_LENGTH) != HAL_OK) {
//            Error_Handler();  // Handle any error during re-enabling
//        }
//    }
}

// Interrupt handler for SPI reception
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

	UartPacket telem;
	telem.id = 0; // arbitrarily deciding that all telem packets have id 0
	telem.packet_type = OW_DATA;
	telem.command = OW_HISTO;
	telem.data_len = SPI_PACKET_LENGTH;
	telem.addr = 0;

	uint32_t xBitToSet = 0x00;
	if(hspi->Instance == SPI2){
		telem.data = pRecieveHistoSpi2;
		telem.id = 6; // cam7
//		UART_INTERFACE_SendDMA(&telem);

		pRecieveHistoSpi2 = (pRecieveHistoSpi2 == spi2RxBufferA) ? spi2RxBufferB : spi2RxBufferA;
		xBitToSet = BIT_6;
		if (HAL_SPI_Receive_DMA(&hspi2, pRecieveHistoSpi2, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	}	else if(hspi->Instance == SPI3){
		telem.data = pRecieveHistoSpi3;
		telem.id = 5;
//		UART_INTERFACE_SendDMA(&telem);
		pRecieveHistoSpi3 = (pRecieveHistoSpi3 == spi3RxBufferA) ? spi3RxBufferB : spi3RxBufferA;
		xBitToSet = BIT_5;
		if (HAL_SPI_Receive_DMA(&hspi3, pRecieveHistoSpi3, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	} else if(hspi->Instance == SPI4){
		telem.data = pRecieveHistoSpi4;
		telem.id = 7;
//		UART_INTERFACE_SendDMA(&telem);

		pRecieveHistoSpi4 = (pRecieveHistoSpi4 == spi4RxBufferA) ? spi4RxBufferB : spi4RxBufferA;
		xBitToSet = BIT_7;

		if (HAL_SPI_Receive_DMA(&hspi4, pRecieveHistoSpi4, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	} else if(hspi->Instance == SPI6){
		telem.data = pRecieveHistoSpi6;
		telem.id = 1;
//		UART_INTERFACE_SendDMA(&telem);

		pRecieveHistoSpi6 = (pRecieveHistoSpi6 == spi6RxBufferA) ? spi6RxBufferB : spi6RxBufferA;
		xBitToSet = BIT_1;
		if (HAL_SPI_Receive_DMA(&hspi6, pRecieveHistoSpi6, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	}
/*
	if (osKernelGetState() != osKernelRunning || event_flags_id == NULL) {
	    printf("Error with posting event flags \r\n");
	}
	else {
		uint32_t flags = osEventFlagsSet(event_flags_id, xBitToSet);
		if (flags == (uint32_t)osFlagsErrorUnknown) {
			printf("osFlagsErrorUnknown!\r\n");
		}
		else if (flags == (uint32_t)osFlagsErrorParameter) {
			printf("osFlagsErrorParameter!\r\n");
		}
		else if (flags == (uint32_t)osFlagsErrorResource) {
			printf("osFlagsErrorResource!\r\n");
		}
	}
	*/
}

// Error handling callback for SPI
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	// Identify specific errors using error codes
	printf("SPI Error occurred: ");
	if (hspi->ErrorCode & HAL_SPI_ERROR_OVR) {
		printf("Overrun error ");
	}
	if (hspi->ErrorCode & HAL_SPI_ERROR_MODF) {
		printf("Mode fault error ");
	}
	if (hspi->ErrorCode & HAL_SPI_ERROR_CRC) {
		printf("CRC error ");
	}
	if (hspi->ErrorCode & HAL_SPI_ERROR_FRE) {
		printf("Frame error ");
	}
	if (hspi->ErrorCode & HAL_SPI_ERROR_DMA) {
		printf("DMA transfer error ");
	}
	printf("\r\n");

	// Reset SPI and buffer state
	HAL_SPI_DeInit(hspi);             // Deinitialize SPI
	Error_Handler();  // Handle any error during re-enabling

/*	HAL_SPI_Init(hspi);               // Reinitialize SPI

	// Re-enable interrupt reception
//    	pRecieveHistoSpi3 = (pRecieveHistoSpi3 == spi3RxBufferA) ? spi3RxBufferB : spi3RxBufferA;
	// Re-enable SPI interrupt reception for the next byte
	if (HAL_SPI_Receive_IT(hspi, pRecieveHistoSpi3, SPI_PACKET_LENGTH) != HAL_OK) {
	}
*/
}


void init_camera(CameraDevice *cam){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Reconfigure CRESETB Pin
	HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
	GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);

	// Reconfigure GPIO0 Pin
	HAL_GPIO_DeInit(cam->gpio0_port, cam->gpio0_pin);
	GPIO_InitStruct.Pin = cam->gpio0_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->gpio0_port, &GPIO_InitStruct);

	/*
	if(cam->useUsart) {
		// Reconfigure USART TX Pin
		HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
		cam->pUart->
		GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
		GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
		HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);
	}
	else {
		// Reconfigure SPI Pin
		HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
		GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
		GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
		HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);
	}
	*/
}
// Task to wait for all bits
/*
void vTaskWaitForAllBits(void *pvParameters)
{
    uint32_t flags;
    for (;;)
    {
        flags = osEventFlagsWait(event_flags_id, ( BIT_5 | BIT_6 | BIT_7 ), osFlagsWaitAll, osWaitForever);

        printf("All bits are set! Task unblocked.\r\n");


//        // Wait for all bits (BIT_0 | BIT_1 | BIT_2) to be set
//        uxBits = xEventGroupWaitBits(
//            xHistoRxEventGroup,
//			BIT_5, // Bits to wait for
//            pdFALSE,           // Do not clear bits on exit
//            pdTRUE,            // Wait for all bits to be set
//            portMAX_DELAY      // Wait indefinitely
//        );
//
//        // Check if all bits are set
//        if ((uxBits & (BIT_5)) == (BIT_5))
//        {
//            printf("\n\n\n\nAll bits are set! Task unblocked.\n\n\n\n");
//        }


    }
}
*/

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM12) {
	  //CDC_Idle_Timer_Handler();
  }


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
