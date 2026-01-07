#include "io_key.h"
#include "bsp_key.h"

/**
 * @brief Initialize the key application layer.
 */
void io_key_init(void) {
    BSP_Key_Init();
    // Reset event flag on init
    (void)BSP_Key_CheckEvent();
}

/**
 * @brief Check if button is currently held down.
 */
bool io_key_is_pressed(void) {
    return (BSP_Key_GetState() == BSP_KEY_PRESSED);
}

/**
 * @brief Check if a button event (press) occurred.
 */
bool io_key_event_occurred(void) {
    return BSP_Key_CheckEvent();
}
