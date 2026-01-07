#include "global.h"
#include "imu_app.h"
#include <stdio.h>
#include "serial_ros.h"

void AppIMUTask(void *argument) {
    (void)argument;
    
    // Initialize IMU
    imu_init(IMU_USE_DEBUG);

    while (1) {
        // Update IMU and publish
        imu_update();

        // Publish IMU data
        imu_data_t imu_raw;
        imu_get_data(&imu_raw);
        
        serial_ros_publish(TOPIC_PUB_IMU, &imu_raw, sizeof(imu_data_t));

        // // Status logging
        // static uint32_t last_log = 0;
        // uint32_t now = osKernelGetTickCount();
        // if (now - last_log >= TIME_LOG_DEBUG_IMU_MS) {
        //     last_log = now;
            
        //     imu_data_t imu;
        //     imu_get_data(&imu);

        //     printf("[TaskIMU] Accel[%.2f, %.2f, %.2f] Gyro[%.2f, %.2f, %.2f] Mag[%.2f, %.2f, %.2f]\r\n", 
        //             imu.acc[0], imu.acc[1], imu.acc[2],
        //             imu.gyro[0], imu.gyro[1], imu.gyro[2],
        //             imu.mag[0], imu.mag[1], imu.mag[2]);
        // }

        osDelay(TIME_IMU_PUBLISH_MS); // 100Hz
    }
}
