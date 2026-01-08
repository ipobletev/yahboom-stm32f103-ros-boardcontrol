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
system_state_t current_state = STATE_TEMPORAL_STOP;
uint32_t global_system_error = SYS_ERROR_NONE;
operation_mode_t current_mode = MODE_MANUAL;
bool is_moving_wheels = false;
bool is_moving_spatial = false;
uint32_t last_cmd_tick = 0;
cmd_vel_t last_cmd = {0};
uint32_t last_conn_ack_tick = 0;
