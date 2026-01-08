#include "global.h"
#include "task_imu.h"
#include "app_debug.h"
#include "imu_app.h"
#include <stdio.h>
#include <string.h>
#include "serial_ros.h"

void AppIMUTask(void *argument) {
    (void)argument;
    
    imu_data_t imu_raw;

    // Initialize IMU
    imu_init(IMU_USE_DEBUG);

    static uint32_t last_publish = 0;
    static uint32_t last_log = 0;

    while (1) {
        // Update IMU (always at loop rate)
        imu_update();

        uint32_t now = osKernelGetTickCount();

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
