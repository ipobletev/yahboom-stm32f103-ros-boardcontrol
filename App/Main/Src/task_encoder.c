#include "app_debug.h"
#include "global.h"
#include "task_encoder.h"
#include <stdio.h>
#include <string.h>
#include "serial_ros.h"
#include "config.h"

int32_t g_encoder_counts[4] = {0};

void AppEncoderTask(void *argument) {
    (void)argument;

    motor_t _motor_fl, _motor_fr, _motor_bl, _motor_br;
    _motor_fl.id = APP_MOTOR_ID_1;
    _motor_fr.id = APP_MOTOR_ID_2;
    _motor_bl.id = APP_MOTOR_ID_3;
    _motor_br.id = APP_MOTOR_ID_4;

    while (1) {
        
        //Get encoders
        int32_t _enc_fl = motor_get_encoder(&_motor_fl);
        int32_t _enc_fr = motor_get_encoder(&_motor_fr);
        int32_t _enc_bl = motor_get_encoder(&_motor_bl);
        int32_t _enc_br = motor_get_encoder(&_motor_br);
        
        // Update global
        g_encoder_counts[0] = _enc_fl;
        g_encoder_counts[1] = _enc_fr;
        g_encoder_counts[2] = _enc_bl;
        g_encoder_counts[3] = _enc_br;

        // Publish encoders
        int32_t enc_data[4] = {_enc_fl, _enc_fr, _enc_bl, _enc_br};
        serial_ros_publish(TOPIC_PUB_ENCODER, enc_data, sizeof(enc_data));
        
        // // Status logging
        static uint32_t _last_log = 0;
        uint32_t _now = osKernelGetTickCount();
        if (_now - _last_log >= TIME_LOG_DEBUG_ENCODER_MS) {
            _last_log = _now;
            APP_DEBUG_INFO("ENCODER", "FL:%ld FR:%ld BL:%ld BR:%ld\r\n",
                (long)_enc_fl, (long)_enc_fr, (long)_enc_bl, (long)_enc_br);
        }

        osDelay(TIME_ENCODER_PUBLISH_MS);
    }
}
