// TIMEOUT CONFIG FOR CMD VEL. If no new command is received for this time, the motors will stop
#define TIMEOUT_LAST_CMD_MS 250

// ROS PUBLISH TIME CONFIG
#define TIME_ENCODER_PUBLISH_MS         100              //10Hz    
#define TIME_IMU_PUBLISH_MS             50               //20Hz
#define TIME_MACHINE_INFO_PUBLISH_MS    250              //4Hz
#define TIME_PID_DEBUG_PUBLISH_MS       50               //20Hz
// STORAGE CONFIG
#define STORAGE_ENABLED                 0                // Set to 1 to enable persistent storage
#define FLASH_STORAGE_ADDR              0x0803F800
#define FLASH_PAGE_SIZE                 2048
#define TIME_WATCHDOG_MS 1000   //1s

//IMU CONFIG
#define IMU_USE_DEBUG true
#define TIME_IMU_UPDATE_MS 20                   //50Hz (Internal update rate)
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
#define SERIAL_ROS_MAX_PAYLOAD 128

// SERIAL ROS TOPIC ID
// Published topics
#define TOPIC_MACHINE_INFO      0x01
#define TOPIC_IMU               0x02
#define TOPIC_ENCODER           0x03
#define TOPIC_PID_DEBUG         0x09
// Subscribed topics
#define TOPIC_CMD_VEL           0x04
#define TOPIC_OPERATION_MODE    0x05
#define TOPIC_OPERATION_RUN     0x06
#define TOPIC_RESET_STOP_CMD    0x07
#define TOPIC_ESTOP_CMD         0x08
#define TOPIC_CONFIRM_CONN      0x0A
#define TOPIC_CONFIG            0x0B

// CONFIG ITEM IDS
#define CONFIG_ITEM_PID_M1      0x00
#define CONFIG_ITEM_PID_M2      0x01
#define CONFIG_ITEM_PID_M3      0x02
#define CONFIG_ITEM_PID_M4      0x03
#define CONFIG_ITEM_WHEEL_DIAM  0x04

// CONNECTION CONFIG
#define CONN_TIMEOUT_MS             3000

// MOTION CONFIG
#define MOTION_FREQ_MS 50
#define ENCODER_THRESHOLD 1
#define GYRO_THRESHOLD 0.05f
#define ACCEL_THRESHOLD 0.1f

// ROBOT PHYSICAL PARAMS
#define WHEEL_DIAMETER 0.080f   // 80mm
#define ENCODER_PPR 1320.0f      // 11 ticks * 30 gear ratio * 4 quadrature ?? TODO: Verify

// ERROR CONFIG
#define APP_USE_HARDWARE_ERROR // If is enabled, the system will stop (EMERGENCY STOP) if a hardware error is detected (global_system_error != 0)

// DEBUG CONFIG
// #define APP_DEBUG_ENABLED                 // Enable debug messages
// LOG DEBUG FOR PUBLISH
#define TIME_LOG_DEBUG_ENCODER_MS 1000
#define TIME_LOG_DEBUG_IMU_MS 1000
#define TIME_LOG_DEBUG_STATE_MS 1000

// #define APP_DISABLE_SERIAL_ROS_PUBLISHERS // DISABLE SERIAL ROS PUBLISHERS ONLY FOR DEVELOPMENT!!!!!!!