#ifndef BSP_LED_H
#define BSP_LED_H

#include "main.h"

/**
 * @brief Initialize LED hardware.
 */
void BSP_LED_Init(void);

/**
 * @brief Turn LED ON.
 */
void BSP_LED_On(void);

/**
 * @brief Turn LED OFF.
 */
void BSP_LED_Off(void);

/**
 * @brief Toggle LED state.
 */
void BSP_LED_Toggle(void);

#endif /* BSP_LED_H */
