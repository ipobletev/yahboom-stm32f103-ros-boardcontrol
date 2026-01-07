#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"

/**
 * @brief Initialize buzzer hardware.
 */
void BSP_Buzzer_Init(void);

/**
 * @brief Turn buzzer ON.
 */
void BSP_Buzzer_On(void);

/**
 * @brief Turn buzzer OFF.
 */
void BSP_Buzzer_Off(void);

/**
 * @brief Toggle buzzer state.
 */
void BSP_Buzzer_Toggle(void);

#endif /* BSP_BUZZER_H */
