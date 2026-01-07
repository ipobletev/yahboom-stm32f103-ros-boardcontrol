#ifndef IO_LED_H
#define IO_LED_H

#include <stdint.h>
#include <stdbool.h>

/** 
 * @brief Initialize all high-level IO components (LED, Buzzer).
 */
void io_led_init(void);

/** 
 * @brief Set LED state.
 * @param on 1 for ON, 0 for OFF.
 */
void io_led_on(bool on);

/**
 * @brief Toggle LED state.
 */
void io_led_toggle(uint16_t duration_ms, bool blocking);

#endif /* IO_LED_H */
