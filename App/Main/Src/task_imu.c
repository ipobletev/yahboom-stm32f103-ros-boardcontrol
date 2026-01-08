#include "global.h"
#include "task_imu.h"
#include "app_debug.h"
#include "imu_app.h"
#include <stdio.h>
#include <string.h>
#include "serial_ros.h"
#include "config.h"

void AppIMUTask(void *argument) {
    (void)argument;
    
    imu_data_t imu_raw;
    static uint32_t last_publish = 0;
    static uint32_t last_log = 0;
    static uint32_t last_health = 0;
    static bool imu_was_ok = false;

    // Initialize IMU
    if (!imu_init(IMU_INIT_RETRIES, IMU_USE_DEBUG)) {
        APP_DEBUG_ERROR("IMU", "IMU initialization failed!");
        global_system_error |= SYS_ERROR_IMU_INIT;
        imu_was_ok = false;
    }

    while (1) {
        uint32_t now = osKernelGetTickCount();

        // Periodic Reinitialization (every 1s) if IMU was not ok
        if (now - last_health >= 1000 && imu_was_ok == false) {
            last_health = now;

            if (!imu_init(IMU_INIT_RETRIES, IMU_USE_DEBUG)) {
                APP_DEBUG_ERROR("IMU", "IMU Reinitialization failed!");
                imu_was_ok = false;
            }else{
                APP_DEBUG_INFO("IMU", "IMU Reinitialization successful!");
                imu_was_ok = true;
            }
            global_system_error &= ~SYS_ERROR_IMU_INIT;
        }

        // Update IMU (always at loop rate)
        imu_update();

        // ROS Publish 
        if (now - last_publish >= TIME_IMU_PUBLISH_MS) {
            last_publish = now;
            
            // Clean and get fresh data for transmission
            memset(&imu_raw, 0, sizeof(imu_data_t));
            imu_get_data(&imu_raw);
            
            serial_ros_publish(TOPIC_PUB_IMU, &imu_raw, sizeof(imu_data_t));

        }

        // Status logging
        if (now - last_log >= TIME_LOG_DEBUG_IMU_MS) {
            last_log = now;

            // If we didn't just publish, we need to get the data for logging
            if (now != last_publish) {
                imu_get_data(&imu_raw);
            }

            APP_DEBUG_INFO("IMU", "Accel[%.2f, %.2f, %.2f] Gyro[%.2f, %.2f, %.2f] Mag[%.2f, %.2f, %.2f]", 
                    imu_raw.acc[0], imu_raw.acc[1], imu_raw.acc[2],
                    imu_raw.gyro[0], imu_raw.gyro[1], imu_raw.gyro[2],
                    imu_raw.mag[0], imu_raw.mag[1], imu_raw.mag[2]);
        }

        osDelay(TIME_IMU_UPDATE_MS); // 100Hz internal update
    }
}
