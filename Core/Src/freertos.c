/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c_sample_adc */
osThreadId_t i2c_sample_adcHandle;
const osThreadAttr_t i2c_sample_adc_attributes = {
  .name = "i2c_sample_adc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c_sample_imu */
osThreadId_t i2c_sample_imuHandle;
const osThreadAttr_t i2c_sample_imu_attributes = {
  .name = "i2c_sample_imu",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adc_sample */
osThreadId_t adc_sampleHandle;
const osThreadAttr_t adc_sample_attributes = {
  .name = "adc_sample",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for service_queue */
osThreadId_t service_queueHandle;
const osThreadAttr_t service_queue_attributes = {
  .name = "service_queue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for can_msg_queue */
osMessageQueueId_t can_msg_queueHandle;
const osMessageQueueAttr_t can_msg_queue_attributes = {
  .name = "can_msg_queue"
};
/* Definitions for i2c_mutex */
osMutexId_t i2c_mutexHandle;
const osMutexAttr_t i2c_mutex_attributes = {
  .name = "i2c_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void start_i2c_adc(void *argument);
void start_i2c_imu(void *argument);
void start_adc_sample(void *argument);
void start_service_queue(void *argument);

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
  /* creation of i2c_mutex */
  i2c_mutexHandle = osMutexNew(&i2c_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of can_msg_queue */
  can_msg_queueHandle = osMessageQueueNew (64, 8, &can_msg_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of i2c_sample_adc */
  i2c_sample_adcHandle = osThreadNew(start_i2c_adc, NULL, &i2c_sample_adc_attributes);

  /* creation of i2c_sample_imu */
  i2c_sample_imuHandle = osThreadNew(start_i2c_imu, NULL, &i2c_sample_imu_attributes);

  /* creation of adc_sample */
  adc_sampleHandle = osThreadNew(start_adc_sample, NULL, &adc_sample_attributes);

  /* creation of service_queue */
  service_queueHandle = osThreadNew(start_service_queue, NULL, &service_queue_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_start_i2c_adc */
/**
* @brief Function implementing the i2c_sample_adc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_i2c_adc */
void start_i2c_adc(void *argument)
{
  /* USER CODE BEGIN start_i2c_adc */
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_i2c_adc */
}

/* USER CODE BEGIN Header_start_i2c_imu */
/**
* @brief Function implementing the i2c_sample_imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_i2c_imu */
void start_i2c_imu(void *argument)
{
  /* USER CODE BEGIN start_i2c_imu */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_i2c_imu */
}

/* USER CODE BEGIN Header_start_adc_sample */
/**
* @brief Function implementing the adc_sample thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_adc_sample */
void start_adc_sample(void *argument)
{
  /* USER CODE BEGIN start_adc_sample */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_adc_sample */
}

/* USER CODE BEGIN Header_start_service_queue */
/**
* @brief Function implementing the service_queue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_service_queue */
void start_service_queue(void *argument)
{
  /* USER CODE BEGIN start_service_queue */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_service_queue */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

