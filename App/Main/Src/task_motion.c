#include "global.h"
#include "task_motion.h"
#include "app_debug.h"
#include "imu_app.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void AppMotionTask(void *argument) {
    (void)argument;

    motor_t motor_fl, motor_fr, motor_bl, motor_br;
    // We only need these for motor_get_encoder, so we initialize them with IDs
    motor_fl.id = MOTOR_ID_1;
    motor_fr.id = MOTOR_ID_2;
    motor_bl.id = MOTOR_ID_3;
    motor_br.id = MOTOR_ID_4;

    int32_t last_enc[4] = {0};
    imu_data_t imu_data;
    
    // Initial read to avoid sudden jump
    last_enc[0] = motor_get_encoder(&motor_fl);
    last_enc[1] = motor_get_encoder(&motor_fr);
    last_enc[2] = motor_get_encoder(&motor_bl);
    last_enc[3] = motor_get_encoder(&motor_br);

    while (1) {
        // 1. Wheel movement detection
        int32_t curr_enc[4];
        curr_enc[0] = motor_get_encoder(&motor_fl);
        curr_enc[1] = motor_get_encoder(&motor_fr);
        curr_enc[2] = motor_get_encoder(&motor_bl);
        curr_enc[3] = motor_get_encoder(&motor_br);

        bool wheels_moving = false;
        for (int i = 0; i < 4; i++) {
            if (abs(curr_enc[i] - last_enc[i]) > ENCODER_THRESHOLD) {
                wheels_moving = true;
                break;
            }
        }
        
        is_moving_wheels = wheels_moving;
        memcpy(last_enc, curr_enc, sizeof(curr_enc));

        // 2. Spatial movement detection
        imu_get_data(&imu_data);
        
        bool spatial_moving = false;
        
        // Check Gyroscope (rotation)
        if (fabs(imu_data.gyro[0]) > GYRO_THRESHOLD || 
            fabs(imu_data.gyro[1]) > GYRO_THRESHOLD || 
            fabs(imu_data.gyro[2]) > GYRO_THRESHOLD) {
            spatial_moving = true;
        }
        
        // Check Accelerometer (vibration/translation)
        // Note: This is a simple threshold on raw accel. 
        // Better would be to compare against gravity or use a high-pass filter if we had more context.
        // For now, let's check for significant deviations from 1g (magnitude) or just changes.
        float accel_mag = sqrtf(imu_data.acc[0]*imu_data.acc[0] + 
                                imu_data.acc[1]*imu_data.acc[1] + 
                                imu_data.acc[2]*imu_data.acc[2]);
        
        if (fabs(accel_mag - 1.0f) > ACCEL_THRESHOLD) {
            spatial_moving = true;
        }

        is_moving_spatial = spatial_moving;

        osDelay(MOTION_FREQ_MS);
    }
}
