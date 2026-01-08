#include "app_debug.h"
#include "bsp_serial_ros.h"
#include "serial_ros.h"
#include "global.h"
#include "task_controller.h"
#include "io_key.h"
#include "io_buzzer.h"
#include <stdio.h>
#include "config.h"

extern cmd_vel_t last_cmd;

#define end_by_emergency_stop() \
    if (current_state == STATE_EMERGENCY_STOP) { \
        APP_DEBUG_INFO("CONTROLLER", "Ignoring command while in E-STOP"); \
        break; \
    }

#define end_by_manual_mode() \
    if (current_mode == MODE_MANUAL) { \
        APP_DEBUG_INFO("CONTROLLER", "Ignoring command while in MANUAL mode"); \
        break; \
    }

// Seriel Reception for ROS commands
static void on_ros_frame_received(uint8_t topic_id, const uint8_t *payload, uint8_t length) {

    APP_DEBUG_INFO("CONTROLLER", "Received ROS frame: topic_id=%d, length=%d", topic_id, length);

    switch (topic_id) {
        case TOPIC_SUB_CMD_VEL:
            end_by_emergency_stop();
            end_by_manual_mode();
            // Only process ROS velocity commands if in AUTONOMOUS mode
            if (length == sizeof(cmd_vel_t)) {
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
            break;

        case TOPIC_SUB_OPERATION_MODE:
            end_by_emergency_stop();
            if (length == 1) {
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
            break;

        case TOPIC_SUB_OPERATION_RUN:
            end_by_emergency_stop();
            end_by_manual_mode();
            if (length == 1) {
                system_state_t target_state = (system_state_t)payload[0];
                if (target_state == STATE_IDLE) {
                    system_msg_t msg = { 
                        .requested_state = STATE_IDLE, 
                        .timestamp = osKernelGetTickCount() 
                    };
                    osMessageQueuePut(system_msg_queue, &msg, 0, 0);
                }
            }
            break;

        case TOPIC_SUB_RESET_STOP_CMD:
            end_by_manual_mode();
            if (length == 1) {
                system_msg_t msg = { 
                    .requested_state = STATE_TEMPORAL_STOP, 
                    .timestamp = osKernelGetTickCount() 
                };
                osMessageQueuePut(system_msg_queue, &msg, 0, 0);
            }
            break;

        case TOPIC_SUB_ESTOP_CMD:
            end_by_emergency_stop();
            end_by_manual_mode();
            if (length == 1) {
                system_msg_t msg = { 
                    .requested_state = STATE_EMERGENCY_STOP, 
                    .timestamp = osKernelGetTickCount() 
                };
                osMessageQueuePut(system_msg_queue, &msg, 0, 0);
            }
            break;

        default:
            APP_DEBUG_INFO("CONTROLLER", "Unknown topic_id: %d", topic_id);
            break;
    }
}

void AppControllerTask(void *argument) {
    (void)argument;
    
    // Register ROS callback for receive messages from serial ROS
    serial_ros_set_callback(on_ros_frame_received);
    
    // Initialize serial ROS BSP (DMA reception)
    serial_ros_bsp_init();
    
    while (1) {
        // Update serial ROS (process incoming bytes)
        serial_ros_update();

        uint32_t now = osKernelGetTickCount();
        
        // 0. Process command timeout
        // If in moving state and last command is older than timeout, stop motors. Secure the system
        if (current_state == STATE_MOVING) {
            if (now - last_cmd_tick > TIMEOUT_LAST_CMD_MS) {
                APP_DEBUG_INFO("CONTROLLER", "Command timeout! Stopping motors.");
                last_cmd.linear_x = 0;
                last_cmd.angular_z = 0;
                system_msg_t msg = { 
                    .requested_state = STATE_IDLE, 
                    .timestamp = now 
                };
                osMessageQueuePut(system_msg_queue, &msg, 0, 0);
            }
        }

        system_msg_t msg;
        msg.timestamp = osKernelGetTickCount();
        bool send_msg = false;

        // 1. Process emergency stop
        //  MANUAL EMERGENCY STOP -> VIA HARDWARE BUTTON
        if (io_key_is_pressed() && current_state != STATE_EMERGENCY_STOP) {
            APP_DEBUG_INFO("CONTROLLER", "Emergency stop! by button. Stopping motors.");
            msg.requested_state = STATE_EMERGENCY_STOP;
            send_msg = true;

        }

        #if APP_USE_HARDWARE_ERROR == 1
        // 2. Process hardware errors
        if (global_system_error != SYS_ERROR_NONE && current_state != STATE_EMERGENCY_STOP) {
            APP_DEBUG_ERROR("CONTROLLER", "Hardware error detected: 0x%08lX! Triggering E-STOP.", global_system_error);
            msg.requested_state = STATE_EMERGENCY_STOP;
            send_msg = true;
        }
        #endif

        // 3. Send message
        if (send_msg) {
            APP_DEBUG_INFO("CONTROLLER", "Sending message to manager: %d", msg.requested_state);
            osMessageQueuePut(system_msg_queue, &msg, 0, 0);
        }

        osDelay(20); // Scan inputs at 50Hz
    }
}


