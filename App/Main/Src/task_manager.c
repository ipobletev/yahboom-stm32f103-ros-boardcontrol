#include "global.h"
#include <stdio.h>
#include "io_buzzer.h"
#include "motor.h"
#include "io_led.h"
#include "serial_ros.h"

osTimerId_t state_pub_timer_id;

void state_moving(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_idle(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_error(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_temporal_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);
void state_emergency_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br);

void StatePubTimerCallback(void *argument)
{
  (void)argument;
  
  machine_info_t payload = {
    .state = current_state,
    .mode = current_mode,
    .is_moving_detected = is_moving_detected
  };
  serial_ros_publish(TOPIC_PUB_MACHINE_INFO, (uint8_t*)&payload, sizeof(payload));
  
}

void AppManagerTask(void *argument) {
    (void)argument;
    
    motor_t motor_fl, motor_fr, motor_bl, motor_br;
    
    // Initialize motors with default config
    motor_init(&motor_fl, NULL, MOTOR_ID_1);
    motor_init(&motor_fr, NULL, MOTOR_ID_2);
    motor_init(&motor_bl, NULL, MOTOR_ID_3);
    motor_init(&motor_br, NULL, MOTOR_ID_4);

    // Initialize and start state publisher timer
    state_pub_timer_id = osTimerNew(StatePubTimerCallback, osTimerPeriodic, NULL, NULL);
    if (state_pub_timer_id != NULL) {
        osTimerStart(state_pub_timer_id, TIME_CURRENT_STATE_PUBLISH_MS);
    }

    while (1) {
        system_msg_t msg;
        // Wait for state change requests (0 timeout to allow periodic logic)
        osStatus_t status = osMessageQueueGet(system_msg_queue, &msg, NULL, 0);

        if (status != osOK) {
            continue;
        }

        //Control logic
        switch (current_state)
        {
            case STATE_IDLE:
                state_idle(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_MOVING:
                state_moving(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_TEMPORAL_STOP:
                state_temporal_stop(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            case STATE_EMERGENCY_STOP: //EMERGENCY STOP
                state_emergency_stop(&motor_fl, &motor_fr, &motor_bl, &motor_br);
                break;
            default:
                break;
        }

        osDelay(10); // Run control loop at 100Hz
    }
}

void state_idle(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    // Stop motors
    motor_set_target_speed(motor_fl, 0);
    motor_set_target_speed(motor_fr, 0);
    motor_set_target_speed(motor_bl, 0);
    motor_set_target_speed(motor_br, 0);
    motor_update(motor_fl);
    motor_update(motor_fr);
    motor_update(motor_bl);
    motor_update(motor_br);
}

void state_moving(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {

    // Check last command to stop motors if timeout of last command
    uint32_t now = osKernelGetTickCount();
    if (current_state == STATE_MOVING && (now - last_cmd_tick > 500)) {
        // printf("[TaskManager] Command timeout! Stopping motors.\r\n");
        last_cmd.linear_x = 0;
        last_cmd.angular_z = 0;
    }

    float left = last_cmd.linear_x - last_cmd.angular_z;
    float right = last_cmd.linear_x + last_cmd.angular_z;
    motor_set_target_speed(motor_fl, left);
    motor_set_target_speed(motor_fr, right);
    motor_set_target_speed(motor_bl, left);
    motor_set_target_speed(motor_br, right);
    motor_update(motor_fl);
    motor_update(motor_fr);
    motor_update(motor_bl);
    motor_update(motor_br);
}

void state_emergency_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    io_buzzer(10);
    motor_stop_all(true);
}

void state_temporal_stop(motor_t *motor_fl, motor_t *motor_fr, motor_t *motor_bl, motor_t *motor_br) {
    io_buzzer(100);
    motor_stop_all(true);
}