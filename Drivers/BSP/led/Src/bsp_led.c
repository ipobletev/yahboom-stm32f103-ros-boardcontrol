#include "bsp_led.h"

void BSP_LED_Init(void) {
    // GPIO is already initialized in MX_GPIO_Init()
    BSP_LED_Off();
}

void BSP_LED_On(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); 
}

void BSP_LED_Off(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void BSP_LED_Toggle(void) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
