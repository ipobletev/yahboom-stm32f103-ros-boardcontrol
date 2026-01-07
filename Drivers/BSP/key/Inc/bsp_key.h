/**
 ******************************************************************************
 * @file    bsp_key.h
 * @brief   Board Support Package for buttons.
 ******************************************************************************
 */

#ifndef BSP_KEY_H
#define BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Key states
 */
typedef enum {
    BSP_KEY_RELEASED = 0,
    BSP_KEY_PRESSED  = 1
} BSP_Key_State;

/**
 * @brief Initialize the key GPIO.
 */
void BSP_Key_Init(void);

/**
 * @brief Read the current state of KEY1.
 * @return BSP_KEY_PRESSED if pressed, BSP_KEY_RELEASED otherwise.
 */
BSP_Key_State BSP_Key_GetState(void);

/**
 * @brief Check if a button event occurred and reset it.
 * @return true if event occurred, false otherwise.
 */
bool BSP_Key_CheckEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
