#include "serial_ros.h"
#include <string.h>
#include "bsp_serial_ros.h"
#include "config.h"
#include "cmsis_os2.h"


typedef enum {
    STATE_WAIT_H1,
    STATE_WAIT_H2,
    STATE_WAIT_TOPIC,
    STATE_WAIT_LEN,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM
} unpack_state_t;

static serial_ros_cb_t ros_callback = NULL;

static uint8_t calculate_checksum(uint8_t topic_id, uint8_t len, const uint8_t *payload) {
    uint8_t checksum = topic_id ^ len;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }
    return checksum;
}

uint16_t serial_ros_pack(uint8_t topic_id, const uint8_t *payload, uint8_t length, uint8_t *buffer) {
    if (length > SERIAL_ROS_MAX_PAYLOAD) return 0;

    buffer[0] = SERIAL_ROS_HEADER_1;
    buffer[1] = SERIAL_ROS_HEADER_2;
    buffer[2] = topic_id;
    buffer[3] = length;
    memcpy(&buffer[4], payload, length);
    buffer[4 + length] = calculate_checksum(topic_id, length, payload);

    return (uint16_t)(5 + length);
}

bool serial_ros_unpack_byte(uint8_t byte, serial_ros_frame_t *out_frame) {
    static unpack_state_t state = STATE_WAIT_H1;
    static uint8_t payload_idx = 0;

    switch (state) {
        case STATE_WAIT_H1:
            if (byte == SERIAL_ROS_HEADER_1) {
                out_frame->header1 = byte;
                state = STATE_WAIT_H2;
            }
            break;

        case STATE_WAIT_H2:
            if (byte == SERIAL_ROS_HEADER_2) {
                out_frame->header2 = byte;
                state = STATE_WAIT_TOPIC;
            } else {
                state = STATE_WAIT_H1;
            }
            break;

        case STATE_WAIT_TOPIC:
            out_frame->topic_id = byte;
            state = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:
            if (byte <= SERIAL_ROS_MAX_PAYLOAD) {
                out_frame->length = byte;
                if (out_frame->length == 0) {
                    state = STATE_WAIT_CHECKSUM;
                } else {
                    payload_idx = 0;
                    state = STATE_WAIT_PAYLOAD;
                }
            } else {
                state = STATE_WAIT_H1;
            }
            break;

        case STATE_WAIT_PAYLOAD:
            out_frame->payload[payload_idx++] = byte;
            if (payload_idx >= out_frame->length) {
                state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM:
            out_frame->checksum = byte;
            state = STATE_WAIT_H1;
            if (out_frame->checksum == calculate_checksum(out_frame->topic_id, out_frame->length, out_frame->payload)) {
                return true;
            }
            break;
    }

    return false;
}

void serial_ros_set_callback(serial_ros_cb_t cb) {
    ros_callback = cb;
}

void serial_ros_publish(uint8_t topic_id, const void *payload, uint8_t length) {
    #ifndef APP_DISABLE_SERIAL_ROS_PUBLISHERS
    static uint8_t buffer[SERIAL_ROS_MAX_PAYLOAD + 5];
    memset(buffer, 0, sizeof(buffer));
    uint16_t total_len = serial_ros_pack(topic_id, (const uint8_t*)payload, length, buffer);
    if (total_len > 0) {
        // Retry mechanism in case UART is busy (e.g. DMA busy)
        for (int i = 0; i < 5; i++) {
            if (serial_ros_bsp_send(buffer, total_len)) {
                break;
            }
            osDelay(1); 
        }
    }
    #endif
}

void serial_ros_update(void) {
    uint8_t byte;
    serial_ros_frame_t frame;
    
    while (serial_ros_bsp_read_byte(&byte)) {
        if (serial_ros_unpack_byte(byte, &frame)) {
            if (ros_callback != NULL) {
                ros_callback(frame.topic_id, (const uint8_t*)frame.payload, frame.length);
            }
        }
    }
}