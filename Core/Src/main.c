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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_comms.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "logging.h"
#include "utils.h"
#include "i2c_master.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

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

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
USART_HandleTypeDef husart1;
USART_HandleTypeDef husart2;
USART_HandleTypeDef husart3;
USART_HandleTypeDef husart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t FIRMWARE_VERSION_DATA[3] = {1, 0, 1};

osThreadId_t comTaskHandle;
const osThreadAttr_t comTask_attributes = {
  .name = "comTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

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
EventGroupHandle_t xHistoRxEventGroup;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI6_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_Init(void);
static void MX_USART3_Init(void);
static void MX_USART6_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_Init(void);
void StartDefaultTask(void *argument);

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
	  printf("IMU detected");
  else printf("IMU detected");


  TCA9548A_SelectChannel(&hi2c1, 0x70, 0);
  HAL_Delay(100);
  I2C_scan(&hi2c1, NULL, 0, true);

  HAL_Delay(1000);


  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  MX_USB_DEVICE_Init();


  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);

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

    HAL_USART_Receive_IT(&husart1, pRecieveHistoUsart1, SPI_PACKET_LENGTH);
    HAL_USART_Receive_IT(&husart2, pRecieveHistoUsart2, SPI_PACKET_LENGTH);
    HAL_USART_Receive_IT(&husart3, pRecieveHistoUsart3, SPI_PACKET_LENGTH);
    HAL_USART_Receive_IT(&husart6, pRecieveHistoUsart6, SPI_PACKET_LENGTH);
    HAL_SPI_Receive_IT(&hspi2, pRecieveHistoSpi2, SPI_PACKET_LENGTH);
  HAL_SPI_Receive_IT(&hspi3, pRecieveHistoSpi3, SPI_PACKET_LENGTH);
  HAL_SPI_Receive_IT(&hspi4, pRecieveHistoSpi4, SPI_PACKET_LENGTH);
  HAL_SPI_Receive_IT(&hspi6, pRecieveHistoSpi6, SPI_PACKET_LENGTH);


  printf("System Running\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  comTaskHandle = osThreadNew(COMTask, NULL, &comTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  xHistoRxEventGroup = xEventGroupCreate();

  xTaskCreate(vTaskWaitForAllBits, "WaitTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_SLAVE;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 24000-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 250-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 4167000;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart1.Init.CLKPhase = USART_PHASE_2EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart1.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart1.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart1, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart1, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 4167000;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart2.Init.CLKPhase = USART_PHASE_2EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart2.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart2.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart2, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart2, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 4167000;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart3.Init.CLKPhase = USART_PHASE_2EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart3.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart3, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart3, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  husart6.Instance = USART6;
  husart6.Init.BaudRate = 4167000;
  husart6.Init.WordLength = USART_WORDLENGTH_8B;
  husart6.Init.StopBits = USART_STOPBITS_1;
  husart6.Init.Parity = USART_PARITY_NONE;
  husart6.Init.Mode = USART_MODE_TX_RX;
  husart6.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart6.Init.CLKPhase = USART_PHASE_2EDGE;
  husart6.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart6.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart6.SlaveMode = USART_SLAVEMODE_ENABLE;
  if (HAL_USART_Init(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart6, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart6, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableFifoMode(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_EnableSlaveMode(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ERROR_LED_Pin|MUX_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FS_OUT_EN_GPIO_Port, FS_OUT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ERROR_LED_Pin MUX_RESET_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin|MUX_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_8_Pin PC12 PC4 IMU_INT_Pin */
  GPIO_InitStruct.Pin = GPIO0_8_Pin|GPIO_PIN_12|GPIO_PIN_4|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_7_Pin GPIO0_5_Pin PB8 */
  GPIO_InitStruct.Pin = GPIO0_7_Pin|GPIO0_5_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_RESET_Pin */
  GPIO_InitStruct.Pin = USB_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_6_Pin CRESET_8_Pin CRESET_7_Pin PE0
                           PE10 PE14 GPIO0_2_Pin GPIO0_3_Pin
                           CRESET_1_Pin CRESET_3_Pin CRESET_2_Pin CRESET_4_Pin */
  GPIO_InitStruct.Pin = GPIO0_6_Pin|CRESET_8_Pin|CRESET_7_Pin|GPIO_PIN_0
                          |GPIO_PIN_10|GPIO_PIN_14|GPIO0_2_Pin|GPIO0_3_Pin
                          |CRESET_1_Pin|CRESET_3_Pin|CRESET_2_Pin|CRESET_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CRESET_6_Pin PD2 PD3 CRESET_5_Pin
                           PD4 PD0 PD15 PD11
                           GPIO0_4_Pin */
  GPIO_InitStruct.Pin = CRESET_6_Pin|GPIO_PIN_2|GPIO_PIN_3|CRESET_5_Pin
                          |GPIO_PIN_4|GPIO_PIN_0|GPIO_PIN_15|GPIO_PIN_11
                          |GPIO0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 GPIO0_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO0_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FSIN_EN_Pin */
  GPIO_InitStruct.Pin = FSIN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSIN_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FS_OUT_EN_Pin */
  GPIO_InitStruct.Pin = FS_OUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FS_OUT_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
//	printf("handler getting hit");

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
        if (HAL_USART_Receive_IT(&husart1, pRecieveHistoUsart1, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART2) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart2;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart2 = (pRecieveHistoUsart2 == usart2RxBufferA) ? usart2RxBufferB : usart2RxBufferA;
        if (HAL_USART_Receive_IT(&husart2, pRecieveHistoUsart2, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART3) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart3;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart3 = (pRecieveHistoUsart3 == usart3RxBufferA) ? usart3RxBufferB : usart3RxBufferA;
        if (HAL_USART_Receive_IT(&husart3, pRecieveHistoUsart3, SPI_PACKET_LENGTH) != HAL_OK) {
            Error_Handler();  // Handle any error during re-enabling
        }
    }
	else if (husart->Instance == USART6) { // Check if the interrupt is for USART2
        telem.data = pRecieveHistoUsart6;
        UART_INTERFACE_SendDMA(&telem);

        pRecieveHistoUsart6 = (pRecieveHistoUsart6 == usart6RxBufferA) ? usart6RxBufferB : usart6RxBufferA;
        if (HAL_USART_Receive_IT(&husart6, pRecieveHistoUsart6, SPI_PACKET_LENGTH) != HAL_OK) {
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
        printf("\n");

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
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t xBitToSet = 0x00;

	if(hspi->Instance == SPI2){
//		telem.data = pRecieveHistoSpi2;
		pRecieveHistoSpi2 = (pRecieveHistoSpi2 == spi2RxBufferA) ? spi2RxBufferB : spi2RxBufferA;
		xBitToSet = BIT_6;
		if (HAL_SPI_Receive_IT(&hspi2, pRecieveHistoSpi2, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	}	else if(hspi->Instance == SPI3){
//		telem.data = pRecieveHistoSpi3;
		pRecieveHistoSpi3 = (pRecieveHistoSpi3 == spi3RxBufferA) ? spi3RxBufferB : spi3RxBufferA;
		xBitToSet = BIT_5;
		if (HAL_SPI_Receive_IT(&hspi3, pRecieveHistoSpi3, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	} else if(hspi->Instance == SPI4){
//		telem.data = pRecieveHistoSpi4;
		pRecieveHistoSpi4 = (pRecieveHistoSpi4 == spi4RxBufferA) ? spi4RxBufferB : spi4RxBufferA;
		xBitToSet = BIT_7;

		if (HAL_SPI_Receive_IT(&hspi4, pRecieveHistoSpi4, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
	} else if(hspi->Instance == SPI6){
//		telem.data = pRecieveHistoSpi6;
		pRecieveHistoSpi6 = (pRecieveHistoSpi6 == spi6RxBufferA) ? spi6RxBufferB : spi6RxBufferA;
		xBitToSet = BIT_1;
		if (HAL_SPI_Receive_IT(&hspi6, pRecieveHistoSpi6, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}

	}

    xResult = xEventGroupSetBitsFromISR(
    								xHistoRxEventGroup,   /* The event group being updated. */
									xBitToSet, /* The bits being set. */
	                                &xHigherPriorityTaskWoken );

	/* Was the message posted successfully? */
	if( xResult != pdFAIL )
	{
		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		   switch should be requested. The macro used is port specific and will
		   be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		   the documentation page for the port being used. */
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

}

// Error handling callback for SPI
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) { // Check if the error is for SPI1
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
        printf("\n");

        // Reset SPI and buffer state
        HAL_SPI_DeInit(hspi);             // Deinitialize SPI
        HAL_SPI_Init(hspi);               // Reinitialize SPI

        // Re-enable interrupt reception
    	pRecieveHistoSpi3 = (pRecieveHistoSpi3 == spi3RxBufferA) ? spi3RxBufferB : spi3RxBufferA;
		// Re-enable SPI interrupt reception for the next byte
		if (HAL_SPI_Receive_IT(&hspi3, pRecieveHistoSpi3, SPI_PACKET_LENGTH) != HAL_OK) {
			Error_Handler();  // Handle any error during re-enabling
		}
    }
}

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;
/* This function is called on each cycle of the idle task. In this case it
does nothing useful, other than report the amout of FreeRTOS heap that
remains unallocated. */
xFreeStackSpace = xPortGetFreeHeapSize();

if( xFreeStackSpace > 100 )
{
/* By now, the kernel has allocated everything it is going to, so
if there is a lot of heap remaining unallocated then
the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
reduced accordingly. */
}

}

void vApplicationMallocFailedHook( void )
{
/* Called if a call to pvPortMalloc() fails because there is insufficient
free memory available in the FreeRTOS heap. pvPortMalloc() is called
internally by FreeRTOS API functions that create tasks, queues, software
timers, and semaphores. The size of the FreeRTOS heap is set by the
configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
( void ) pcTaskName;
( void ) pxTask;

/* Run time stack overflow checking is performed if
configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook
function is called if a stack overflow is detected. */
for( ;; );
}

void COMTask(void *argument)
{
	  printf("Starting COM Task\r\n");
	  comms_start_task();
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
void vTaskWaitForAllBits(void *pvParameters)
{
    EventBits_t uxBits;

    for (;;)
    {
        printf("Task is waiting for all bits to be set...\n");

        // Wait for all bits (BIT_0 | BIT_1 | BIT_2) to be set
        uxBits = xEventGroupWaitBits(
            xHistoRxEventGroup,
			BIT_0 | BIT_1 | BIT_2, // Bits to wait for
            pdFALSE,           // Do not clear bits on exit
            pdTRUE,            // Wait for all bits to be set
            portMAX_DELAY      // Wait indefinitely
        );

        // Check if all bits are set
        if ((uxBits & (BIT_0 | BIT_1 | BIT_2)) == (BIT_0 | BIT_1 | BIT_2))
        {
            printf("All bits are set! Task unblocked.\n");
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
	  CDC_Idle_Timer_Handler();
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
