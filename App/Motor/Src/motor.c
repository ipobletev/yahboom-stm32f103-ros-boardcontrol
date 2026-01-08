#include "motor.h"
#include "bsp_motor.h"
#include "pid.h"
#include <stdio.h>

#define MOTOR_MAX_PULSE 3600
#define MOTOR_IGNORE_PULSE 10

static int16_t motor_apply_dead_zone(int16_t pulse) {
    if (pulse < 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse > 0) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

static int16_t motor_limit_speed(int16_t pulse) {
    if (pulse > MOTOR_MAX_PULSE) return MOTOR_MAX_PULSE;
    if (pulse < -MOTOR_MAX_PULSE) return -MOTOR_MAX_PULSE;
    return pulse;
}

bool motor_init(motor_t *motor, const motor_config_t *config, app_motor_id_t id) {
    if (motor == NULL) return false;

    static bool bsp_initialized = false;
    if (!bsp_initialized) {
        BSP_Motor_Init();
        bsp_initialized = true;
    }

    motor->id = id;
    
    if (config != NULL) {
        motor->use_pid = config->use_pid;
        pid_init(&motor->pid, config->pid.kp, config->pid.ki, config->pid.kd, config->pid.max_output, config->pid.min_output);
    } else {
        printf("[Motor] ERROR: Invalid motor config for motor %d\r\n", id);
        return false;
    }
    
    motor->last_encoder = 0;
    motor->target_speed = 0;
    return true;
}

void motor_set_speed(motor_t *motor, int16_t speed) {
    if (motor == NULL) return;
    
    int16_t pulse = motor_apply_dead_zone(speed);
    pulse = motor_limit_speed(pulse);
    
    motor->target_speed = 0; 
    BSP_Motor_Set_Pwm((uint8_t)motor->id, pulse);
}

void motor_stop_all(uint8_t brake) {
    // This stops all motors via hardware
    BSP_Motor_Stop(brake);
}

void motor_stop(motor_t *motor, uint8_t brake) {
    if (motor == NULL) return;
    motor->target_speed = 0;
    pid_reset(&motor->pid);
    BSP_Motor_Set_Pwm((uint8_t)motor->id, brake ? MOTOR_MAX_PULSE : 0);
}

int32_t motor_get_encoder(motor_t *motor) {
    if (motor == NULL) return 0;
    return BSP_Motor_Get_Encoder((uint8_t)motor->id);
}

void motor_reset_encoder(motor_t *motor) {
    if (motor == NULL) return;
    BSP_Motor_Reset_Encoder((uint8_t)motor->id);
    motor->last_encoder = 0;
}

void motor_set_target_speed(motor_t *motor, float speed) {
    if (motor != NULL) {
        motor->target_speed = speed;
    }
}

void motor_set_pid_gains(motor_t *motor, float kp, float ki, float kd) {
    if (motor != NULL) {
        motor->pid.Kp = kp;
        motor->pid.Ki = ki;
        motor->pid.Kd = kd;
    }
}

void motor_update(motor_t *motor) {
    if (motor == NULL) return;

    // Calculate current speed (pulses since last update)
    int32_t current_enc = BSP_Motor_Get_Encoder((uint8_t)motor->id);
    float current_speed = (float)(current_enc - motor->last_encoder);
    motor->last_encoder = current_enc;
    
    // If target is 0 and we are not moving much, just stop
    if (motor->target_speed == 0 && current_speed == 0) {
        BSP_Motor_Set_Pwm((uint8_t)motor->id, 0);
        pid_reset(&motor->pid);
        return;
    }
    
    // Calculate PID or use target directly if PID is disabled
    float output;
    if (motor->use_pid) {
        output = pid_calculate(&motor->pid, motor->target_speed, current_speed);
    } else {
        output = motor->target_speed;
    }
    
    // Apply output
    int16_t pulse = (int16_t)output;
    pulse = motor_apply_dead_zone(pulse);
    pulse = motor_limit_speed(pulse);
    
    BSP_Motor_Set_Pwm((uint8_t)motor->id, pulse);
}
