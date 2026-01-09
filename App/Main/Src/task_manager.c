#include "app_debug.h"
#include "task_manager.h"
#include "global.h"
#include "imu_app.h"
#include "bsp_adc.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "io_buzzer.h"
#include "motor.h"
#include "io_led.h"
#include "serial_ros.h"
#include "config.h"
#include "storage.h"

osTimerId_t state_pub_timer_id;
osTimerId_t pid_debug_timer_id;

void state_moving(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_idle(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_error(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_temporal_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_emergency_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);

void PidDebugTimerCallback(void *argument)
{
    pid_debug_msg_t msg = {
        .target = {motor_fl.pid.target, motor_fr.pid.target, motor_bl.pid.target, motor_br.pid.target},
        .current = {motor_fl.pid.current, motor_fr.pid.current, motor_bl.pid.current, motor_br.pid.current},
        .error = {motor_fl.pid.error[0], motor_fr.pid.error[0], motor_bl.pid.error[0], motor_br.pid.error[0]}
    };
    serial_ros_publish(TOPIC_PID_DEBUG, (uint8_t*)&msg, sizeof(msg));
}


void StatePubTimerCallback(void *argument)
{
  imu_data_t imu_raw;
  imu_get_data(&imu_raw);

  // Calculate Inclination
  float roll = atan2f(imu_raw.acc[1], imu_raw.acc[2]) * 57.29578f;
  float pitch = atan2f(-imu_raw.acc[0], sqrtf(imu_raw.acc[1] * imu_raw.acc[1] + imu_raw.acc[2] * imu_raw.acc[2])) * 57.29578f;

  // Calculate Velocity from Encoders
  static int32_t last_encs[4] = {0};
  static uint32_t last_time = 0;
  uint32_t now = osKernelGetTickCount();
  float dt = (now - last_time) / 1000.0f;
  
  float velocity = 0.0f;
  
  if (dt > 0.0f && last_time > 0) {
      int32_t delta_sum = 0;
      for(int i=0; i<4; i++) {
        delta_sum += (g_encoder_counts[i] - last_encs[i]);
      }
      float avg_delta = (float)delta_sum / 4.0f;
      // v = (delta / dt) * (PI * D / PPR)
      velocity = (avg_delta / dt) * (3.14159f * g_wheel_diameter / ENCODER_PPR);
  }
  
  // Update history
  for(int i=0; i<4; i++) last_encs[i] = g_encoder_counts[i];
  last_time = now;

  machine_info_t payload = {
    .state = current_state,
    .mode = current_mode,
    .is_moving_wheels = is_moving_wheels,
    .is_moving_spatial = is_moving_spatial,
    .error_code = global_system_error,
    .roll = roll,
    .pitch = pitch,
    .velocity = velocity,
    .battery = bsp_adc_battery_read(),
    .temperature = bsp_adc_temperature_read(),
    .angular_velocity = (fabs(imu_raw.gyro[2]) > GYRO_THRESHOLD) ? imu_raw.gyro[2] : 0.0f
  };
  serial_ros_publish(TOPIC_MACHINE_INFO, (uint8_t*)&payload, sizeof(payload));
  
  APP_DEBUG_INFO("MANAGER", "State: %d, Mode: %d, Moving (W:%d, S:%d), Error: %d, Roll: %.1f, Pitch: %.1f, Vel: %.2f, AngVel: %.2f, Bat: %.1fV, Temp: %.1fC\r\n", 
          payload.state, payload.mode, payload.is_moving_wheels, payload.is_moving_spatial, payload.error_code,
          payload.roll, payload.pitch, payload.velocity, payload.angular_velocity, payload.battery, payload.temperature);

}

void AppManagerTask(void *argument) {
    (void)argument;

    //Initialize buzzer
    io_buzzer_init();
    
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

    // Load persistent config if available
#if STORAGE_ENABLED
    app_config_t storage_cfg;
    if (storage_load(&storage_cfg)) {
        APP_DEBUG_INFO("MANAGER", "Applying persistent configuration from Flash");
        g_wheel_diameter = storage_cfg.wheel_diameter;
        motor_set_pid_gains(&motor_fl, storage_cfg.pid[0].kp, storage_cfg.pid[0].ki, storage_cfg.pid[0].kd);
        motor_set_pid_gains(&motor_fr, storage_cfg.pid[1].kp, storage_cfg.pid[1].ki, storage_cfg.pid[1].kd);
        motor_set_pid_gains(&motor_bl, storage_cfg.pid[2].kp, storage_cfg.pid[2].ki, storage_cfg.pid[2].kd);
        motor_set_pid_gains(&motor_br, storage_cfg.pid[3].kp, storage_cfg.pid[3].ki, storage_cfg.pid[3].kd);
    }
#endif

    if (!m_ok) {
        APP_DEBUG_ERROR("MANAGER", "Motor initialization failed!");
        global_system_error |= SYS_ERROR_MOTOR_INIT;
    }

    // Initialize and start timers
    state_pub_timer_id = osTimerNew(StatePubTimerCallback, osTimerPeriodic, NULL, NULL);
    if (state_pub_timer_id != NULL) {
        osTimerStart(state_pub_timer_id, TIME_MACHINE_INFO_PUBLISH_MS);
        APP_DEBUG_INFO("MANAGER", "State publisher timer started (%d ms)", TIME_MACHINE_INFO_PUBLISH_MS);
    }

    pid_debug_timer_id = osTimerNew(PidDebugTimerCallback, osTimerPeriodic, NULL, NULL);
    if (pid_debug_timer_id != NULL) {
        osTimerStart(pid_debug_timer_id, TIME_PID_DEBUG_PUBLISH_MS);
        APP_DEBUG_INFO("MANAGER", "PID debug timer started (%d ms)", TIME_PID_DEBUG_PUBLISH_MS);
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