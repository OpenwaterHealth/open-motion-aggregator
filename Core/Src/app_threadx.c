/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD ux_comms_thread;

TX_QUEUE	queue_0;
TX_SEMAPHORE semaphore_0;
TX_MUTEX mutex_0;
TX_EVENT_FLAGS_GROUP    event_flags_id;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void tx_comms_thread_entry(ULONG thread_input);

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  UCHAR *pointer;

  if(tx_byte_allocate(byte_pool, (VOID **)&pointer, 512, TX_NO_WAIT) != TX_SUCCESS)
  {
	  return TX_POOL_ERROR;
  }
  if(tx_thread_create(&ux_comms_thread, "Comms Thread", tx_comms_thread_entry, 1, pointer, 512, 20, 20, 1, TX_AUTO_START) != TX_SUCCESS)
  {
	  return TX_THREAD_ERROR;
  }
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */


  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
void tx_comms_thread_entry(ULONG thread_input)
{
	UINT    status;

	printf("Starting COM Task\r\n");
	comms_start_task();
}

/* USER CODE END 1 */
