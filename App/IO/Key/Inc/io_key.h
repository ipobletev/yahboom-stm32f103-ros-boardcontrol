/**
 ******************************************************************************
 * @file    io_key.h
 * @brief   Application level IO interface for buttons.
 ******************************************************************************
 */

#ifndef IO_KEY_H
#define IO_KEY_H

#include <stdbool.h>

/**
 * @brief Initialize the key application layer.
 */
void io_key_init(void);

/**
 * @brief Get button press event.
 * @return true if a new press was detected.
 */
bool io_key_is_pressed(void);

/**
 * @brief Check if a button event (press) occurred.
 */
bool io_key_event_occurred(void);

#endif /* IO_KEY_H */
