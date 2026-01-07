#ifndef WDG_APP_H
#define WDG_APP_H

#include <stdint.h>

/**
 * @brief Initialize the Watchdog Application layer.
 *        This will start a software timer to refresh the IWDG.
 * @param interval_ms Refresh interval in milliseconds.
 */
void wdg_app_init(uint32_t interval_ms);

#endif /* WDG_APP_H */
