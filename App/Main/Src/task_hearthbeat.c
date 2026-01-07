#include "cmsis_os2.h"
#include "global.h"
#include <stdio.h>
#include "io_buzzer.h"
#include "io_led.h"
#include "io_key.h"
#include "imu_app.h"
#include "wdg_app.h"
#include "motor.h"
#include "global.h"
#include "app_tasks.h"


/* 
    Test task
*/
void AppHearthbeatTask(void *argument) {
    (void)argument;
    
    // io_buzzer_init();
    io_led_init();
    // imu_init(IMU_USE_DEBUG); //True is for debug mode

    // motor_config_t motor_1_cfg = {
    //     .use_pid = MOTOR_1_USE_PID,
    //     .pid = {.kp = MOTOR_1_PID_KP, .ki = MOTOR_1_PID_KI, .kd = MOTOR_1_PID_KD, .max_output = MOTOR_1_PID_MAX_OUTPUT, .min_output = MOTOR_1_PID_MIN_OUTPUT}
    // };
    // motor_config_t motor_2_cfg = {
    //     .use_pid = MOTOR_2_USE_PID,
    //     .pid = {.kp = MOTOR_2_PID_KP, .ki = MOTOR_2_PID_KI, .kd = MOTOR_2_PID_KD, .max_output = MOTOR_2_PID_MAX_OUTPUT, .min_output = MOTOR_2_PID_MIN_OUTPUT}
    // };
    // motor_config_t motor_3_cfg = {
    //     .use_pid = MOTOR_3_USE_PID,
    //     .pid = {.kp = MOTOR_3_PID_KP, .ki = MOTOR_3_PID_KI, .kd = MOTOR_3_PID_KD, .max_output = MOTOR_3_PID_MAX_OUTPUT, .min_output = MOTOR_3_PID_MIN_OUTPUT}
    // };
    // motor_config_t motor_4_cfg = {
    //     .use_pid = MOTOR_4_USE_PID,
    //     .pid = {.kp = MOTOR_4_PID_KP, .ki = MOTOR_4_PID_KI, .kd = MOTOR_4_PID_KD, .max_output = MOTOR_4_PID_MAX_OUTPUT, .min_output = MOTOR_4_PID_MIN_OUTPUT}
    // };
    // motor_t motors[4];

    // motor_init(&motors[0], &motor_1_cfg, MOTOR_ID_1);
    // motor_init(&motors[1], &motor_2_cfg, MOTOR_ID_2);
    // motor_init(&motors[2], &motor_3_cfg, MOTOR_ID_3);
    // motor_init(&motors[3], &motor_4_cfg, MOTOR_ID_4);

    //micro_ros_app_init();

    // printf("Application layer initialized\r\n");

    while (1) {

        // printf("Application layer running\r\n");

        //micro_ros_app_run();
        
        // static uint32_t last_print_tick = 0;
        // static float t_speed = 0.0f; // pulses per 10ms
        // static int8_t t_dir = 1;
        // static uint32_t last_target_update = 0;

        // uint32_t current_tick = osKernelGetTickCount();
        
        // // Update all motors
        // for (int i = 0; i < 4; i++) {
        //     motor_update(&motors[i]);
        // }

        // // Get encoder data
        // int32_t enc[4];
        // for (int i = 0; i < 4; i++) {
        //     enc[i] = motor_get_encoder(&motors[i]);
        // }

        // // Print IMU and Motor data every second
        // if (current_tick - last_print_tick >= 1000) {
        //     imu_data_t imu;
        //     imu_get_data(&imu);
        //     last_print_tick = current_tick;
        //     printf("IMU: A[%.2f,%.2f,%.2f] G[%.2f,%.2f,%.2f]\r\n",
        //         imu.acc[0], imu.acc[1], imu.acc[2],
        //         imu.gyro[0], imu.gyro[1], imu.gyro[2]);
            
        //     // Log Motor PID performance
        //     printf("Motor PID: T1[%.2f] T2[%.2f] T3[%.2f] T4[%.2f] E1[%ld] E2[%ld] E3[%ld] E4[%ld]\r\n", 
        //         t_speed, t_speed, t_speed, t_speed, (long)enc[0], (long)enc[1], (long)enc[2], (long)enc[3]);
        // }

        // // Check for button press/hold
        // if (io_key_is_pressed()) {
        //     // Toggle Buzzer non-blockingly
        //     io_buzzer(50);
        // }

        // Toggle LED non-blockingly
        io_led_toggle(10, false);

        // // Test logic: Update target speed every 5 seconds
        // if (current_tick - last_target_update >= 5000) {
        //     last_target_update = current_tick;
        //     t_speed += 10.0f * t_dir;
        //     if (t_speed > 50.0f || t_speed < -50.0f) t_dir = -t_dir;
            
        //     for (int i = 0; i < 4; i++) {
        //         motor_set_target_speed(&motors[i], t_speed);
        //     }
        //     printf("Target speed set to: %.2f\r\n", t_speed);
        // }

        osDelay(500);
    }
}
