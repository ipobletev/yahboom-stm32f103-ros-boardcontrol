#ifndef IO_BUZZER_H
#define IO_BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize all high-level IO components (LED, Buzzer).
 */
void io_buzzer_init(void);

/**
 * @brief Beep the buzzer for a short duration (non-blocking).
 * @param duration_ms Duration in milliseconds.
 */
void io_buzzer(uint16_t duration_ms);

#endif /* IO_BUZZER_H */
