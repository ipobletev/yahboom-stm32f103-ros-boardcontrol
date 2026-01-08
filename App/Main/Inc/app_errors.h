#ifndef APP_ERRORS_H
#define APP_ERRORS_H

#include <stdint.h>

/**
 * @file app_errors.h
 * @brief System-wide error codes and bitmask definitions.
 */

/* No error */
#define SYS_ERROR_NONE              0x00000000

/* Hardware Initialization Errors (Bits 0-7) */
#define SYS_ERROR_IMU_INIT          (1 << 0)
#define SYS_ERROR_MOTOR_INIT        (1 << 1)
#define SYS_ERROR_ENCODER_INIT      (1 << 2)
#define SYS_ERROR_SERIAL_ROS_INIT   (1 << 3)

/* Runtime Communication Errors (Bits 8-15) */
#define SYS_ERROR_IMU_TIMEOUT       (1 << 8)
#define SYS_ERROR_SERIAL_TIMEOUT    (1 << 9)
#define SYS_ERROR_CMD_TIMEOUT       (1 << 10)

/* Resource Errors (Bits 16-23) */
#define SYS_ERROR_QUEUE_FULL        (1 << 16)
#define SYS_ERROR_TIMER_FAIL        (1 << 17)
#define SYS_ERROR_MALLOC_FAIL       (1 << 18)

/* Critical System Errors (Bits 24-31) */
#define SYS_ERROR_WATCHDOG          (1 << 24)
#define SYS_ERROR_BROWNOUT          (1 << 25)
#define SYS_ERROR_OVERTEMPERATURE   (1 << 26)

#endif /* APP_ERRORS_H */
