#include "io_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include <stdio.h>

static osTimerId_t buzzerTimerHandle;

static void buzzerTimerCallback(void *argument) {
    (void)argument;
    BSP_Buzzer_Off();
}

// IO initialization
void io_buzzer_init(void) {
    BSP_Buzzer_Init();

    // Create a single-shot timer for the buzzer
    buzzerTimerHandle = osTimerNew(buzzerTimerCallback, osTimerOnce, NULL, NULL);
    if (buzzerTimerHandle == NULL) {
        printf("Failed to create buzzer timer\r\n");
    }
}

// Buzzer control
void io_buzzer(uint16_t duration_ms) {
    if (buzzerTimerHandle != NULL) {
        osTimerStop(buzzerTimerHandle);
        BSP_Buzzer_On();
        osTimerStart(buzzerTimerHandle, pdMS_TO_TICKS(duration_ms));
    }
}
