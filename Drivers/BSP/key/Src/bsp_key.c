/**
 ******************************************************************************
 * @file    bsp_key.c
 * @brief   Implementation of BSP for buttons.
 ******************************************************************************
 */

#include "bsp_key.h"

/**
 * @brief Initialize the key GPIO.
 * @note GPIO is already initialized in MX_GPIO_Init() in gpio.c.
 */
void BSP_Key_Init(void) {
    // Clock and GPIO init handled by MX_GPIO_Init()
}


static volatile bool button_event_flag = false;

/**
 * @brief Read the current state of KEY1.
 * @return BSP_KEY_PRESSED if pressed, BSP_KEY_RELEASED otherwise.
 */
BSP_Key_State BSP_Key_GetState(void) {
    // KEY1 is active low (pulled up, grounded when pressed)
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        return BSP_KEY_PRESSED;
    }
    return BSP_KEY_RELEASED;
}

/**
 * @brief Check if a button event occurred and reset it.
 */
bool BSP_Key_CheckEvent(void) {
    if (button_event_flag) {
        button_event_flag = false;
        return true;
    }
    return false;
}

/**
 * @brief HAL GPIO EXTI Callback.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY1_Pin) {
        button_event_flag = true;
    }
}
