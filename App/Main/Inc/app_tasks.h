#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "task_controller.h"
#include "task_encoder.h"
#include "task_hearthbeat.h"
#include "task_imu.h"
#include "task_manager.h"
#include "task_motion.h"

typedef StaticTask_t osStaticThreadDef_t;

// Task Handles
extern osThreadId_t testTaskHandle;
extern osThreadId_t controllerTaskHandle;
extern osThreadId_t managerTaskHandle;
extern osThreadId_t heartbeatTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t encoderTaskHandle;
extern osThreadId_t motionTaskHandle;

/**
 * @brief Test task: Sets up hardware. (Strong implementation of weak TestTask)
 */
void TestTask(void *argument);

#endif // APP_TASKS_H
