/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_uart3.h"
#include "user_can.h"
#include "user_adc.h"
#include "user_motor.h"
#include "database.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for uart3RxTask */
osThreadId_t uart3RxTaskHandle;
const osThreadAttr_t uart3RxTask_attributes = {
  .name = "uart3RxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for uart3TxTask */
osThreadId_t uart3TxTaskHandle;
const osThreadAttr_t uart3TxTask_attributes = {
  .name = "uart3TxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTransferTask */
osThreadId_t canTransferTaskHandle;
const osThreadAttr_t canTransferTask_attributes = {
  .name = "canTransferTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for canReceiveData */
osMessageQueueId_t canReceiveDataHandle;
const osMessageQueueAttr_t canReceiveData_attributes = {
  .name = "canReceiveData"
};
/* Definitions for canTransferData */
osMessageQueueId_t canTransferDataHandle;
const osMessageQueueAttr_t canTransferData_attributes = {
  .name = "canTransferData"
};
/* Definitions for myMutexSendQueue */
osMutexId_t myMutexSendQueueHandle;
const osMutexAttr_t myMutexSendQueue_attributes = {
  .name = "myMutexSendQueue"
};
/* Definitions for uart3RxBinarySem */
osSemaphoreId_t uart3RxBinarySemHandle;
const osSemaphoreAttr_t uart3RxBinarySem_attributes = {
  .name = "uart3RxBinarySem"
};
/* Definitions for ADC_IRQ_BinarySem */
osSemaphoreId_t ADC_IRQ_BinarySemHandle;
const osSemaphoreAttr_t ADC_IRQ_BinarySem_attributes = {
  .name = "ADC_IRQ_BinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartCanTask(void *argument);
void StartUart3RxTask(void *argument);
void StartUart3TxTask(void *argument);
void StartadcTaskTask(void *argument);
void StartCanTransferTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myMutexSendQueue */
  myMutexSendQueueHandle = osMutexNew(&myMutexSendQueue_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart3RxBinarySem */
  uart3RxBinarySemHandle = osSemaphoreNew(1, 0, &uart3RxBinarySem_attributes);

  /* creation of ADC_IRQ_BinarySem */
  ADC_IRQ_BinarySemHandle = osSemaphoreNew(1, 0, &ADC_IRQ_BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  if(uart3RxBinarySemHandle == NULL || ADC_IRQ_BinarySemHandle == NULL)
  {
	  while(1)
	  {

	  }
  }
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canReceiveData */
  canReceiveDataHandle = osMessageQueueNew (5, 12, &canReceiveData_attributes);

  /* creation of canTransferData */
  canTransferDataHandle = osMessageQueueNew (20, 12, &canTransferData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  while(canReceiveDataHandle == NULL)
  {

  }
  while(canTransferDataHandle == NULL)
  {

  }
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* creation of canTask */
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);

  /* creation of uart3RxTask */
  uart3RxTaskHandle = osThreadNew(StartUart3RxTask, NULL, &uart3RxTask_attributes);

  /* creation of uart3TxTask */
  uart3TxTaskHandle = osThreadNew(StartUart3TxTask, NULL, &uart3TxTask_attributes);

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(StartadcTaskTask, NULL, &adcTask_attributes);

  /* creation of canTransferTask */
  canTransferTaskHandle = osThreadNew(StartCanTransferTask, NULL, &canTransferTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
    static TickType_t Main_tick;

    Main_tick = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  adj_motor_handler();
	  basedata_queue_handler();
	  node_guarding_check();
	  vTaskDelayUntil(&Main_tick, pdMS_TO_TICKS(5));//绝对延时
//	  osDelay(10);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  /* Infinite loop */
  for(;;)
  {
	  user_can_RX_handler();
//    osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartUart3RxTask */
/**
* @brief Function implementing the uart3RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart3RxTask */
void StartUart3RxTask(void *argument)
{
  /* USER CODE BEGIN StartUart3RxTask */
	osSemaphoreAcquire(uart3RxBinarySemHandle, 0);
	user_uart3_init();
  /* Infinite loop */
  for(;;)
  {
	  uart3_rx_handler();
//    osDelay(1);
  }
  /* USER CODE END StartUart3RxTask */
}

/* USER CODE BEGIN Header_StartUart3TxTask */
/**
* @brief Function implementing the uart3TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart3TxTask */
void StartUart3TxTask(void *argument)
{
  /* USER CODE BEGIN StartUart3TxTask */
    static TickType_t tick;

    tick = xTaskGetTickCount();
    osDelay(10);
  /* Infinite loop */
	for(;;)
	{
#if TEST_CAN_USART == 0
		uart3_tx_handler();
#endif
		vTaskDelayUntil(&tick, pdMS_TO_TICKS(1));//绝对延时
	}
  /* USER CODE END StartUart3TxTask */
}

/* USER CODE BEGIN Header_StartadcTaskTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartadcTaskTask */
void StartadcTaskTask(void *argument)
{
  /* USER CODE BEGIN StartadcTaskTask */
	osSemaphoreAcquire(ADC_IRQ_BinarySemHandle, 0);
	adc_start();//�???????始采�???????

  /* Infinite loop */
  for(;;)
  {
	  adc_buff_handler();
//    osDelay(1);
  }
  /* USER CODE END StartadcTaskTask */
}

/* USER CODE BEGIN Header_StartCanTransferTask */
/**
* @brief Function implementing the canTransferTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTransferTask */
void StartCanTransferTask(void *argument)
{
  /* USER CODE BEGIN StartCanTransferTask */
  /* Infinite loop */
  for(;;)
  {
	  CAN_transfer_queue_handler();
//    osDelay(5);
  }
  /* USER CODE END StartCanTransferTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

