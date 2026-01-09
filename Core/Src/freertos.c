/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "global.h"
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
/* Definitions for AppController */
osThreadId_t AppControllerHandle;
const osThreadAttr_t AppController_attributes = {
  .name = "AppController",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AppEncoder */
osThreadId_t AppEncoderHandle;
const osThreadAttr_t AppEncoder_attributes = {
  .name = "AppEncoder",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for AppIMU */
osThreadId_t AppIMUHandle;
const osThreadAttr_t AppIMU_attributes = {
  .name = "AppIMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AppManager */
osThreadId_t AppManagerHandle;
const osThreadAttr_t AppManager_attributes = {
  .name = "AppManager",
  .stack_size = 1048 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for AppHearthbeat */
osThreadId_t AppHearthbeatHandle;
const osThreadAttr_t AppHearthbeat_attributes = {
  .name = "AppHearthbeat",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AppMotion */
osThreadId_t AppMotionHandle;
const osThreadAttr_t AppMotion_attributes = {
  .name = "AppMotion",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppControllerTask(void *argument);
void AppEncoderTask(void *argument);
void AppIMUTask(void *argument);
void AppManagerTask(void *argument);
void AppHearthbeatTask(void *argument);
void AppMotionTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  system_msg_queue = osMessageQueueNew(10, sizeof(system_msg_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AppController */
  AppControllerHandle = osThreadNew(AppControllerTask, NULL, &AppController_attributes);

  /* creation of AppEncoder */
  AppEncoderHandle = osThreadNew(AppEncoderTask, NULL, &AppEncoder_attributes);

  /* creation of AppIMU */
  AppIMUHandle = osThreadNew(AppIMUTask, NULL, &AppIMU_attributes);

  /* creation of AppManager */
  AppManagerHandle = osThreadNew(AppManagerTask, NULL, &AppManager_attributes);

  /* creation of AppHearthbeat */
  AppHearthbeatHandle = osThreadNew(AppHearthbeatTask, NULL, &AppHearthbeat_attributes);

  /* creation of AppMotion */
  AppMotionHandle = osThreadNew(AppMotionTask, NULL, &AppMotion_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_AppControllerTask */
/**
  * @brief  Function implementing the AppController thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AppControllerTask */
__weak void AppControllerTask(void *argument)
{
  /* USER CODE BEGIN AppControllerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppControllerTask */
}

/* USER CODE BEGIN Header_AppEncoderTask */
/**
* @brief Function implementing the AppEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppEncoderTask */
__weak void AppEncoderTask(void *argument)
{
  /* USER CODE BEGIN AppEncoderTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppEncoderTask */
}

/* USER CODE BEGIN Header_AppIMUTask */
/**
* @brief Function implementing the AppIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppIMUTask */
__weak void AppIMUTask(void *argument)
{
  /* USER CODE BEGIN AppIMUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppIMUTask */
}

/* USER CODE BEGIN Header_AppManagerTask */
/**
* @brief Function implementing the AppManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppManagerTask */
__weak void AppManagerTask(void *argument)
{
  /* USER CODE BEGIN AppManagerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppManagerTask */
}

/* USER CODE BEGIN Header_AppHearthbeatTask */
/**
* @brief Function implementing the AppHearthbeat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppHearthbeatTask */
__weak void AppHearthbeatTask(void *argument)
{
  /* USER CODE BEGIN AppHearthbeatTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppHearthbeatTask */
}

/* USER CODE BEGIN Header_AppMotionTask */
/**
* @brief Function implementing the AppMotion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppMotionTask */
__weak void AppMotionTask(void *argument)
{
  /* USER CODE BEGIN AppMotionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppMotionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

