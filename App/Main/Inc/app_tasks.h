#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

typedef StaticTask_t osStaticThreadDef_t;

// Task Handles
extern osThreadId_t testTaskHandle;
extern osThreadId_t controllerTaskHandle;
extern osThreadId_t managerTaskHandle;
extern osThreadId_t heartbeatTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t encoderTaskHandle;

/**
 * @brief Test task: Sets up hardware. (Strong implementation of weak TestTask)
 */
void TestTask(void *argument);

/**
 * @brief Controller task: Monitors inputs and sends state change requests.
 */
void AppControllerTask(void *argument);

/**
 * @brief Manager task: Receives state change requests and manages system behavior.
 */
void AppManagerTask(void *argument);

/**
 * @brief IMU task: Periodically updates and publishes IMU data.
 */
void AppIMUTask(void *argument);

/**
 * @brief Encoder task: Periodically reads and publishes encoder data.
 */
void AppEncoderTask(void *argument);

#endif // APP_TASKS_H
