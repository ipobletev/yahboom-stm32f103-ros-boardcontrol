#include "io_led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include <stdio.h>

static osTimerId_t ledTimerHandle;

static void ledTimerCallback(void *argument) {
    (void)argument;
    BSP_LED_Off();
}

// IO initialization
void io_led_init(void) {
    BSP_LED_Init();

    ledTimerHandle = osTimerNew(ledTimerCallback, osTimerOnce, NULL, NULL);
    if (ledTimerHandle == NULL) {
        printf("Failed to create LED timer\r\n");
    }
}


// LED control
void io_led_on(bool on) {
    if (on) BSP_LED_On();
    else BSP_LED_Off();
}

void io_led_toggle(uint16_t duration_ms, bool blocking) {
    if (blocking) {
        if (ledTimerHandle != NULL) {
            osTimerStop(ledTimerHandle);
        }
        BSP_LED_On();
        osDelay(pdMS_TO_TICKS(duration_ms));
        BSP_LED_Off();
    } else {
        if (ledTimerHandle != NULL) {
            osTimerStop(ledTimerHandle);
            BSP_LED_On();
            osTimerStart(ledTimerHandle, pdMS_TO_TICKS(duration_ms));
        }
    }
}