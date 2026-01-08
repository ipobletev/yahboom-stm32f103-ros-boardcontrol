#include "app_debug.h"
#include "global.h"
#include "task_encoder.h"
#include <stdio.h>
#include <string.h>
#include "serial_ros.h"
#include "config.h"

void AppEncoderTask(void *argument) {
    (void)argument;

    motor_t _motor_fl, _motor_fr, _motor_bl, _motor_br;

    while (1) {

        //clean
        memset(&_motor_fl, 0, sizeof(motor_t));
        memset(&_motor_fr, 0, sizeof(motor_t));
        memset(&_motor_bl, 0, sizeof(motor_t));
        memset(&_motor_br, 0, sizeof(motor_t));
        
        //Get encoders
        int32_t _enc_fl = motor_get_encoder(&_motor_fl);
        int32_t _enc_fr = motor_get_encoder(&_motor_fr);
        int32_t _enc_bl = motor_get_encoder(&_motor_bl);
        int32_t _enc_br = motor_get_encoder(&_motor_br);
        // fake encoders
        _enc_fl += 1;
        _enc_fr += 1;
        _enc_bl += 1;
        _enc_br += 1;

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
