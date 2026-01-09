#include "cmsis_os2.h"
#include "task_hearthbeat.h"
#include "global.h"
#include <stdio.h>
#include "io_buzzer.h"
#include "io_led.h"
#include "io_key.h"
#include "imu_app.h"
#include "wdg_app.h"
#include "motor.h"
#include "global.h"
#include "app_debug.h"
#include "app_tasks.h"

/* 
    Heartbeat task
*/
void AppHearthbeatTask(void *argument) {
    (void)argument;

    wdg_app_init(1000);
    io_led_init();

    while (1) {

        // Toggle LED non-blockingly
        io_led_toggle(10, false);

        osDelay(500);
    }
}
