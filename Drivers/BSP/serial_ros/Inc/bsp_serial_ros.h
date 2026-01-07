#ifndef SERIAL_ROS_BSP_H
#define SERIAL_ROS_BSP_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the serial hardware.
 */
void serial_ros_bsp_init(void);

/**
 * @brief Send raw bytes over the serial interface.
 */
void serial_ros_bsp_send(const uint8_t *data, uint16_t length);

/**
 * @brief Read a single byte from the serial interface (non-blocking).
 * @return true if a byte was read, false otherwise.
 */
bool serial_ros_bsp_read_byte(uint8_t *byte);

#endif // SERIAL_ROS_BSP_H
