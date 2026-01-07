#include "app_debug.h"
#include "global.h"
#include <stdio.h>
#include <string.h>
#include "serial_ros.h"

void AppEncoderTask(void *argument) {
    (void)argument;

    //Initialize motors
    motor_t motor_fl, motor_fr, motor_bl, motor_br;

     // Initialize motors with default config
    motor_init(&motor_fl, NULL, MOTOR_ID_1);
    motor_init(&motor_fr, NULL, MOTOR_ID_2);
    motor_init(&motor_bl, NULL, MOTOR_ID_3);
    motor_init(&motor_br, NULL, MOTOR_ID_4);

    while (1) {

        //clean
        memset(&motor_fl, 0, sizeof(motor_t));
        memset(&motor_fr, 0, sizeof(motor_t));
        memset(&motor_bl, 0, sizeof(motor_t));
        memset(&motor_br, 0, sizeof(motor_t));
        
        //Get encoders
        int32_t enc_fl = motor_get_encoder(&motor_fl);
        int32_t enc_fr = motor_get_encoder(&motor_fr);
        int32_t enc_bl = motor_get_encoder(&motor_bl);
        int32_t enc_br = motor_get_encoder(&motor_br);
        //fake encoders
        // int32_t enc_fl = 0;
        // int32_t enc_fr = 0;
        // int32_t enc_bl = 0;
        // int32_t enc_br = 0;

        // // Publish encoders
        // int32_t enc_data[4] = {enc_fl, enc_fr, enc_bl, enc_br};
        // serial_ros_publish(TOPIC_PUB_ENCODER, enc_data, sizeof(enc_data));
        
        // // Status logging
        static uint32_t last_log = 0;
        uint32_t now = osKernelGetTickCount();
        if (now - last_log >= TIME_LOG_DEBUG_ENCODER_MS) {
            last_log = now;
            APP_DEBUG_INFO("ENCODER", "FL:%ld FR:%ld BL:%ld BR:%ld\r\n",
                (long)enc_fl, (long)enc_fr, (long)enc_bl, (long)enc_br);
            }

        osDelay(TIME_ENCODER_PUBLISH_MS);
    }
}
