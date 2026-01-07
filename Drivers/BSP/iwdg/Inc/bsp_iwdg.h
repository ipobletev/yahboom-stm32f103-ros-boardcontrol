#ifndef BSP_IWDG_H
#define BSP_IWDG_H

#include "main.h"

/**
 * @brief Initialize the Independent Watchdog.
 */
void bsp_iwdg_init(void);

/**
 * @brief Refresh the Independent Watchdog counter.
 */
void bsp_iwdg_refresh(void);

#endif /* BSP_IWDG_H */
