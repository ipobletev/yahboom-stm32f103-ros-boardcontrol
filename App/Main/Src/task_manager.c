#include "app_debug.h"
#include "task_manager.h"
#include "global.h"
#include <stdio.h>
#include <string.h>
#include "io_buzzer.h"
#include "motor.h"
#include "io_led.h"
#include "serial_ros.h"

osTimerId_t state_pub_timer_id;

void state_moving(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_idle(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_error(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_temporal_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_emergency_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);


void StatePubTimerCallback(void *argument)
{
  (void)argument;
  
  machine_info_t payload = {
    .state = current_state,
    .mode = current_mode,
    .is_moving_wheels = is_moving_wheels,
    .is_moving_spatial = is_moving_spatial,
    .error_code = global_system_error
  };
  serial_ros_publish(TOPIC_PUB_MACHINE_INFO, (uint8_t*)&payload, sizeof(payload));
  
  APP_DEBUG_INFO("MANAGER", "State: %d, Mode: %d, Moving (W:%d, S:%d)\r\n", 
          payload.state, payload.mode, payload.is_moving_wheels, payload.is_moving_spatial);

}

void AppManagerTask(void *argument) {
    (void)argument;

    //Initialize buzzer
    io_buzzer_init();
    
    //Initialize motors
    motor_t motor_fl, motor_fr, motor_bl, motor_br;
    //clean
    memset(&motor_fl, 0, sizeof(motor_t));
    memset(&motor_fr, 0, sizeof(motor_t));
    memset(&motor_bl, 0, sizeof(motor_t));
    memset(&motor_br, 0, sizeof(motor_t));
    
    // Initialize motors with default config
    motor_config_t motor_fl_cfg = {
        .use_pid = MOTOR_1_USE_PID,
        .pid = {
            .kp = MOTOR_1_PID_KP, 
            .ki = MOTOR_1_PID_KI, 
            .kd = MOTOR_1_PID_KD, 
            .max_output = MOTOR_1_PID_MAX_OUTPUT, 
            .min_output = MOTOR_1_PID_MIN_OUTPUT
        }
    };
    motor_config_t motor_fr_cfg = {
        .use_pid = MOTOR_2_USE_PID,
        .pid = {
            .kp = MOTOR_2_PID_KP, 
            .ki = MOTOR_2_PID_KI, 
            .kd = MOTOR_2_PID_KD, 
            .max_output = MOTOR_2_PID_MAX_OUTPUT, 
            .min_output = MOTOR_2_PID_MIN_OUTPUT
        }
    };
    motor_config_t motor_bl_cfg = {
        .use_pid = MOTOR_3_USE_PID,
        .pid = {
            .kp = MOTOR_3_PID_KP, 
            .ki = MOTOR_3_PID_KI, 
            .kd = MOTOR_3_PID_KD, 
            .max_output = MOTOR_3_PID_MAX_OUTPUT, 
            .min_output = MOTOR_3_PID_MIN_OUTPUT
        }
    };
    motor_config_t motor_br_cfg = {
        .use_pid = MOTOR_4_USE_PID,
        .pid = {
            .kp = MOTOR_4_PID_KP, 
            .ki = MOTOR_4_PID_KI, 
            .kd = MOTOR_4_PID_KD, 
            .max_output = MOTOR_4_PID_MAX_OUTPUT, 
            .min_output = MOTOR_4_PID_MIN_OUTPUT
        }
    };

    bool m_ok = true;
    m_ok &= motor_init(&motor_fl, &motor_fl_cfg, MOTOR_ID_1);
    m_ok &= motor_init(&motor_fr, &motor_fr_cfg, MOTOR_ID_2);
    m_ok &= motor_init(&motor_bl, &motor_bl_cfg, MOTOR_ID_3);
    m_ok &= motor_init(&motor_br, &motor_br_cfg, MOTOR_ID_4);

    if (!m_ok) {
        APP_DEBUG_ERROR("MANAGER", "Motor initialization failed!");
        global_system_error |= SYS_ERROR_MOTOR_INIT;
    }

    // Initialize and start state publisher timer
    state_pub_timer_id = osTimerNew(StatePubTimerCallback, osTimerPeriodic, NULL, NULL);
    if (state_pub_timer_id != NULL) {
        osTimerStart(state_pub_timer_id, TIME_MACHINE_INFO_PUBLISH_MS);
        APP_DEBUG_INFO("MANAGER", "State publisher timer started (%d ms)", TIME_MACHINE_INFO_PUBLISH_MS);
    } else {
        APP_DEBUG_ERROR("MANAGER", "Failed to create state publisher timer");
    }

    while (1) {
        system_msg_t msg;
        // Wait for state change requests
        osStatus_t status = osMessageQueueGet(system_msg_queue, &msg, NULL, 0);

        if (status == osOK) {
            APP_DEBUG_INFO("MANAGER", "State changed to: %d", msg.requested_state);
            current_state = msg.requested_state;
        }

        //Control logic
        switch (current_state)
        {
            case STATE_IDLE:
                state_idle(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_MOVING:
                state_moving(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_TEMPORAL_STOP:
                state_temporal_stop(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_EMERGENCY_STOP:
                state_emergency_stop(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            default:
                break;
        }

        osDelay(10); // Run control loop at 100Hz
    }
}

void state_idle(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    // Stop motors
    motor_set_target_speed(motor_fl, 0);
    motor_set_target_speed(motor_fr, 0);
    motor_set_target_speed(motor_bl, 0);
    motor_set_target_speed(motor_br, 0);
    motor_update(motor_fl);
    motor_update(motor_fr);
    motor_update(motor_bl);
    motor_update(motor_br);
}

void state_moving(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {

    float left = last_cmd.linear_x - last_cmd.angular_z;
    float right = last_cmd.linear_x + last_cmd.angular_z;
    motor_set_target_speed(motor_fl, left);
    motor_set_target_speed(motor_fr, right);
    motor_set_target_speed(motor_bl, left);
    motor_set_target_speed(motor_br, right);
    motor_update(motor_fl);
    motor_update(motor_fr);
    motor_update(motor_bl);
    motor_update(motor_br);
    
}

void state_emergency_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    static uint32_t last_beep = 0;
    uint32_t now = osKernelGetTickCount();
    
    // Beep for 100ms every 500ms
    if (now - last_beep > 500) {
        io_buzzer(100);
        last_beep = now;
    }
    motor_stop_all(true);
}

void state_temporal_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    static uint32_t last_beep = 0;
    uint32_t now = osKernelGetTickCount();
    
    // Beep for 100ms every 500ms
    if (now - last_beep > 2000) {
        io_buzzer(100);
        last_beep = now;
    }
    motor_stop_all(true);
}