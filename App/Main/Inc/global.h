#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "motor.h"

#define APP_DEBUG_ENABLED 0

// ROS PUBLISH TIME CONFIG
#define TIME_ENCODER_PUBLISH_MS 20           //50Hz    
#define TIME_IMU_PUBLISH_MS 20               //50Hz
#define TIME_CURRENT_STATE_PUBLISH_MS 1000   //1s

// LOG DEBUG FOR PUBLISH
#define TIME_LOG_DEBUG_ENCODER_MS 1000
#define TIME_LOG_DEBUG_IMU_MS 1000
#define TIME_LOG_DEBUG_STATE_MS 1000

// WATCHDOG CONFIG
#define TIME_WATCHDOG_MS 1000   //1s

//IMU CONFIG
#define IMU_USE_DEBUG true
#define TIME_IMU_UPDATE_MS 10                   //100Hz (Internal update rate)

// MOTOR CONFIG
#define MOTOR_ID_1 0
#define MOTOR_ID_2 1
#define MOTOR_ID_3 2
#define MOTOR_ID_4 3

// MOTOR PID CONFIG
#define MOTOR_1_USE_PID true
#define MOTOR_1_PID_KP 1.5f
#define MOTOR_1_PID_KI 0.5f
#define MOTOR_1_PID_KD 0.0f
#define MOTOR_1_PID_MAX_OUTPUT 3600.0f
#define MOTOR_1_PID_MIN_OUTPUT -3600.0f

#define MOTOR_2_USE_PID true
#define MOTOR_2_PID_KP 1.5f
#define MOTOR_2_PID_KI 0.5f
#define MOTOR_2_PID_KD 0.0f
#define MOTOR_2_PID_MAX_OUTPUT 3600.0f
#define MOTOR_2_PID_MIN_OUTPUT -3600.0f

#define MOTOR_3_USE_PID true
#define MOTOR_3_PID_KP 1.5f
#define MOTOR_3_PID_KI 0.5f
#define MOTOR_3_PID_KD 0.0f
#define MOTOR_3_PID_MAX_OUTPUT 3600.0f
#define MOTOR_3_PID_MIN_OUTPUT -3600.0f

#define MOTOR_4_USE_PID true
#define MOTOR_4_PID_KP 1.5f
#define MOTOR_4_PID_KI 0.5f
#define MOTOR_4_PID_KD 0.0f
#define MOTOR_4_PID_MAX_OUTPUT 3600.0f
#define MOTOR_4_PID_MIN_OUTPUT -3600.0f

// SERIAL ROS CONFIG
#define SERIAL_ROS_HEADER_1 0xAA
#define SERIAL_ROS_HEADER_2 0x55
#define SERIAL_ROS_MAX_PAYLOAD 64

// SERIAL ROS TOPIC ID
// Published topics
#define TOPIC_PUB_MACHINE_INFO      0x01
#define TOPIC_PUB_IMU               0x02
#define TOPIC_PUB_ENCODER           0x03
// Subscribed topics
#define TOPIC_SUB_CMD_VEL           0x04
#define TOPIC_SUB_OPERATION_MODE    0x05
#define TOPIC_SUB_OPERATION_RUN     0x06

// MOTION CONFIG
#define MOTION_FREQ_MS 50
#define ENCODER_THRESHOLD 1
#define GYRO_THRESHOLD 0.05f
#define ACCEL_THRESHOLD 0.1f

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
    //future implementation
        //inclination
        //velocity
        //battery
        //temperature
        //error_code
} machine_info_t;

/* Shared global variables */
extern osMessageQueueId_t system_msg_queue;
extern system_state_t current_state;
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
