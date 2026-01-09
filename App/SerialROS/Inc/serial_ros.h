#ifndef SERIAL_ROS_H
#define SERIAL_ROS_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifndef SERIAL_ROS_HEADER_1
#define SERIAL_ROS_HEADER_1 0xAA
#endif

#ifndef SERIAL_ROS_HEADER_2
#define SERIAL_ROS_HEADER_2 0x55
#endif

#ifndef SERIAL_ROS_MAX_PAYLOAD
#define SERIAL_ROS_MAX_PAYLOAD 64
#endif

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t topic_id;
    uint8_t length;
    uint8_t payload[SERIAL_ROS_MAX_PAYLOAD];
    uint8_t checksum;
} serial_ros_frame_t;

/**
 * @brief Callback function type for received ROS frames.
 */
typedef void (*serial_ros_cb_t)(uint8_t topic_id, const uint8_t *payload, uint8_t length);

/**
 * @brief Pack data into a serial frame.
 * @return Total frame length if successful, 0 otherwise.
 */
uint16_t serial_ros_pack(uint8_t topic_id, const uint8_t *payload, uint8_t length, uint8_t *buffer);

/**
 * @brief Unpack a single byte into the state machine.
 * @return True if a complete frame has been received.
 */
bool serial_ros_unpack_byte(uint8_t byte, serial_ros_frame_t *out_frame);

/**
 * @brief Set the callback for received ROS frames.
 */
void serial_ros_set_callback(serial_ros_cb_t cb);

/**
 * @brief Publish data to a specific topic.
 */
void serial_ros_publish(uint8_t topic_id, const void *payload, uint8_t length);

/**
 * @brief Update serial ROS state machine and handle subscriptions.
 */
void serial_ros_update(void);

#endif // SERIAL_ROS_H
