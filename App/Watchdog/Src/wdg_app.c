#include "wdg_app.h"
#include "bsp_iwdg.h"
#include "cmsis_os2.h"

static osTimerId_t wdg_timer_id;

/**
 * @brief Watchdog refresh timer callback.
 */
static void wdg_timer_callback(void *argument) {
    (void)argument;
    bsp_iwdg_refresh();
}

void wdg_app_init(uint32_t interval_ms) {
    // Initialize BSP
    bsp_iwdg_init();

    // Create software timer ONLY if it doesn't exist
    if (wdg_timer_id == NULL) {
        wdg_timer_id = osTimerNew(wdg_timer_callback, osTimerPeriodic, NULL, NULL);
    }

    if (wdg_timer_id != NULL) {
        // Restart or Start timer with the provided interval
        osTimerStart(wdg_timer_id, interval_ms);
    }
}
