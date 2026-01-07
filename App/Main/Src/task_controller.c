#include "bsp_serial_ros.h"
#include "serial_ros.h"
#include "global.h"
#include "io_key.h"
#include "io_buzzer.h"
#include <stdio.h>

extern cmd_vel_t last_cmd;

// Seriel Reception for ROS commands
static void on_ros_frame_received(uint8_t topic_id, const uint8_t *payload, uint8_t length) {

    // 1. Process ROS velocity commands
    if (topic_id == TOPIC_SUB_CMD_VEL && length == sizeof(cmd_vel_t)) {
        // Only process ROS velocity commands if in AUTONOMOUS mode
        if (current_mode == MODE_AUTONOMOUS) {
            cmd_vel_t *cmd = (cmd_vel_t*)payload;
            last_cmd = *cmd; // Update global command
            last_cmd_tick = osKernelGetTickCount();

            if (current_state == STATE_IDLE && (cmd->linear_x != 0 || cmd->angular_z != 0)) {
                system_msg_t msg = { 
                    .requested_state = STATE_MOVING, 
                    .timestamp = last_cmd_tick 
                };
                osMessageQueuePut(system_msg_queue, &msg, 0, 0);
            }
        }
    }

    // 2. Process ROS mode commands
    else if (topic_id == TOPIC_SUB_OPERATION_MODE && length == 1) {
        operation_mode_t new_mode = (operation_mode_t)payload[0];
        if (new_mode == MODE_MANUAL || new_mode == MODE_AUTONOMOUS) {
            current_mode = new_mode;
            // Stop command when switching modes
            last_cmd.linear_x = 0;
            last_cmd.angular_z = 0;
            
            system_msg_t msg = { 
                .requested_state = STATE_TEMPORAL_STOP, 
                .timestamp = osKernelGetTickCount() 
            };
            osMessageQueuePut(system_msg_queue, &msg, 0, 0);
        }
    }
    // 3. Process a Run device (to make ready to move. STATE_IDLE)
    else if (topic_id == TOPIC_SUB_OPERATION_RUN && length == 1) {
        if (payload[0] == 0) {
            system_msg_t msg = { 
                .requested_state = STATE_IDLE, 
                .timestamp = osKernelGetTickCount() 
            };
            osMessageQueuePut(system_msg_queue, &msg, 0, 0);
        }
    }
}

void AppControllerTask(void *argument) {
    (void)argument;
    
    // Register ROS callback for receive messages from serial ROS
    serial_ros_set_callback(on_ros_frame_received);
    
    while (1) {
        system_msg_t msg;
        msg.timestamp = osKernelGetTickCount();
        bool send_msg = false;

        // 1. Process emergency stop
        //  MANUAL EMERGENCY STOP -> VIA HARDWARE BUTTON
        if (io_key_is_pressed() && current_state != STATE_EMERGENCY_STOP) {
            msg.requested_state = STATE_EMERGENCY_STOP;
            send_msg = true;
        }

        // 2. Send message
        if (send_msg) {
            osMessageQueuePut(system_msg_queue, &msg, 0, 0);
        }

        osDelay(20); // Scan inputs at 50Hz
    }
}
