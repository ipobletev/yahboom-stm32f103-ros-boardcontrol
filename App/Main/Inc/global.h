#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "motor.h"
#include "app_errors.h"

/**
 * @brief Operation modes for the application.
 */
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTONOMOUS = 1
} operation_mode_t;

/**
 * @brief System states for the application.
 */
typedef enum {
    STATE_IDLE,
    STATE_MOVING,
    STATE_TEMPORAL_STOP,
    STATE_EMERGENCY_STOP
} system_state_t;

/**
 * @brief System error flags (Refer to app_errors.h for definitions).
 */

/**
 * @brief System messages for the state queue.
 */
typedef struct {
    system_state_t requested_state;
    uint32_t timestamp;
} system_msg_t;

/**
 * @brief Machine information for the application.
 */
typedef struct {
    system_state_t state;
    operation_mode_t mode;
    bool is_moving_wheels;
    bool is_moving_spatial;
    uint32_t error_code;
    float roll;      // inclination roll (deg)
    float pitch;     // inclination pitch (deg)
    float velocity;  // linear velocity (m/s)
    float battery;   // battery voltage (V)
    float temperature; // internal temperature (degC)
} machine_info_t;

extern machine_info_t machine_info;
extern int32_t g_encoder_counts[4]; 

/* Shared global variables */
extern osMessageQueueId_t system_msg_queue;
extern system_state_t current_state;
extern uint32_t global_system_error;
extern operation_mode_t current_mode;
extern bool is_moving_wheels;
extern bool is_moving_spatial;
extern uint32_t last_cmd_tick;

typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_z;
} cmd_vel_t;

extern cmd_vel_t last_cmd;

// /* Shared motor instances */
// extern motor_t motor_fl, motor_fr, motor_bl, motor_br;

#endif // APP_H
