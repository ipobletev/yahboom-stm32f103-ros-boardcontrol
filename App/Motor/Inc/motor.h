#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float min_output;
} motor_pid_params_t;

typedef struct {
    bool use_pid;
    motor_pid_params_t pid;
} motor_config_t;

typedef enum {
    APP_MOTOR_ID_1 = 0,
    APP_MOTOR_ID_2 = 1,
    APP_MOTOR_ID_3 = 2,
    APP_MOTOR_ID_4 = 3
} app_motor_id_t;

/**
 * @brief Motor instance structure.
 */
typedef struct {
    app_motor_id_t id;
    pid_control_t pid;
    bool use_pid;
    int32_t last_encoder;
    float target_speed;
} motor_t;

/**
 * @brief Initialize a motor instance.
 * @param motor Pointer to the motor structure to initialize.
 * @param config Pointer to the configuration.
 * @param id Motor hardware identifier.
 */
bool motor_init(motor_t *motor, const motor_config_t *config, app_motor_id_t id);

/**
 * @brief Set motor speed (open loop).
 * @param motor Pointer to the motor instance.
 * @param speed Speed value.
 */
void motor_set_speed(motor_t *motor, int16_t speed);

/**
 * @brief Stop all motors (global) or a specific one? 
 * Let's keep a global stop but also individual stops.
 */
void motor_stop_all(uint8_t brake);
void motor_stop(motor_t *motor, uint8_t brake);

/**
 * @brief Get motor encoder count.
 */
int32_t motor_get_encoder(motor_t *motor);

/**
 * @brief Reset motor encoder count.
 */
void motor_reset_encoder(motor_t *motor);

/**
 * @brief Set motor target speed for PID control.
 */
void motor_set_target_speed(motor_t *motor, float speed);

/**
 * @brief Update motor control loop for a specific instance.
 */
void motor_update(motor_t *motor);

/**
 * @brief Set PID gains for a specific motor.
 */
void motor_set_pid_gains(motor_t *motor, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
