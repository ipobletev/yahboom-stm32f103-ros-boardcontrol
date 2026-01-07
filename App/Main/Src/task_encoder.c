#include "global.h"
#include <stdio.h>
#include "serial_ros.h"

#define TIME_ENCODER_UPDATE_MS 20       //50Hz

void AppEncoderTask(void *argument) {
    (void)argument;

    while (1) {

        //Get encoders
        // int32_t enc_fl = motor_get_encoder(&motor_fl);
        // int32_t enc_fr = motor_get_encoder(&motor_fr);
        // int32_t enc_bl = motor_get_encoder(&motor_bl);
        // int32_t enc_br = motor_get_encoder(&motor_br);
        //fake encoders
        int32_t enc_fl = 0;
        int32_t enc_fr = 0;
        int32_t enc_bl = 0;
        int32_t enc_br = 0;

        // Publish encoders
        int32_t enc_data[4] = {enc_fl, enc_fr, enc_bl, enc_br};
        serial_ros_publish(TOPIC_PUB_ENCODER, enc_data, sizeof(enc_data));
        
        // Status logging
        static uint32_t last_log = 0;
        uint32_t now = osKernelGetTickCount();
        if (now - last_log >= TIME_LOG_DEBUG_ENCODER_MS) {
            last_log = now;

            printf("[TaskEncoder] FL:%ld FR:%ld BL:%ld BR:%ld\r\n", 
                    (long)enc_fl, (long)enc_fr, (long)enc_bl, (long)enc_br);
        }

        osDelay(TIME_ENCODER_UPDATE_MS);
    }
}
