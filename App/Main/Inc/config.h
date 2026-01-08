// DEBUG CONFIG
#define APP_DEBUG_ENABLED 0
#define APP_USE_HARDWARE_ERROR 0 // If hardware error is enabled, the system will stop (EMERGENCY STOP) if a hardware error is detected

// TIMEOUT CONFIG FOR CMD VEL. If no new command is received for this time, the motors will stop
#define TIMEOUT_LAST_CMD_MS 250

// ROS PUBLISH TIME CONFIG
#define TIME_ENCODER_PUBLISH_MS         1000              //20Hz    
#define TIME_IMU_PUBLISH_MS             1000            //100Hz
#define TIME_MACHINE_INFO_PUBLISH_MS    1000           //1s

// LOG DEBUG FOR PUBLISH
#define TIME_LOG_DEBUG_ENCODER_MS 1000
#define TIME_LOG_DEBUG_IMU_MS 1000
#define TIME_LOG_DEBUG_STATE_MS 1000

// WATCHDOG CONFIG
#define TIME_WATCHDOG_MS 1000   //1s

//IMU CONFIG
#define IMU_USE_DEBUG true
#define TIME_IMU_UPDATE_MS 10                   //100Hz (Internal update rate)
#define IMU_INIT_RETRIES 5

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
#define TOPIC_SUB_RESET_STOP_CMD    0x07
#define TOPIC_SUB_ESTOP_CMD         0x08

// MOTION CONFIG
#define MOTION_FREQ_MS 50
#define ENCODER_THRESHOLD 1
#define GYRO_THRESHOLD 0.05f
#define ACCEL_THRESHOLD 0.1f