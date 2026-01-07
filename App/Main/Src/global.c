#include "global.h"
#include <stdio.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "wdg_app.h"
#include "motor.h"
#include "io_buzzer.h"
#include "io_led.h"
#include "io_key.h"
#include "imu_app.h"

/* Shared status variables (Global) */
osMessageQueueId_t system_msg_queue = NULL;
system_state_t current_state = STATE_INIT;
operation_mode_t current_mode = MODE_MANUAL;
bool is_moving_detected = false;
uint32_t last_cmd_tick = 0;
cmd_vel_t last_cmd = {0};

/**
 * @brief Thread-safe state transitions
 */
void set_system_state(system_state_t new_state) {
    if (current_state == new_state) return;

    switch (new_state) {
        case STATE_EMERGENCY_STOP:
            io_buzzer(500);
            printf("[App] ENTERING EMERGENCY STOP\r\n");
            break;
        case STATE_TEMPORAL_STOP:
            io_buzzer(500);
            printf("[App] ENTERING TEMPORAL STOP\r\n");
            break;
        case STATE_IDLE:
            printf("[App] Transiting to IDLE\r\n");
            break;
        case STATE_MOVING:
            printf("[App] Transiting to MOVING\r\n");
            break;
        default:
            break;
    }
    current_state = new_state;
}
