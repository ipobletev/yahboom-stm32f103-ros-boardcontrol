/**
 * @file app_debug.h
 * @brief Multi-level tagged debug printing utility.
 */

#ifndef __APP_DEBUG_H__
#define __APP_DEBUG_H__

#include <stdio.h>
#include "config.h"

/**
 * @brief Master switch to enable/disable all debug output.
 * Set to 1 to enable, 0 to disable.
 */
#ifndef APP_DEBUG_ENABLED
#define APP_DEBUG_ENABLED 0
#endif

/**
 * @brief Debug level definitions.
 */
typedef enum {
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_WARN,
    DEBUG_LEVEL_ERROR
} debug_level_t;

/**
 * @brief Internal function to handle the formatted print.
 * Do not call this directly, use the macros instead.
 */
void app_debug_print(const char* tag, debug_level_t level, const char* fmt, ...);

/**
 * @brief Main debug macro.
 */
#if (APP_DEBUG_ENABLED == 1)
    #define APP_DEBUG(tag, level, fmt, ...) app_debug_print(tag, level, fmt, ##__VA_ARGS__)
#else
    #define APP_DEBUG(tag, level, fmt, ...) ((void)0)
#endif

/**
 * @brief Helper macros for different levels.
 */
#define APP_DEBUG_INFO(tag, fmt, ...)  APP_DEBUG(tag, DEBUG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define APP_DEBUG_WARN(tag, fmt, ...)  APP_DEBUG(tag, DEBUG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define APP_DEBUG_ERROR(tag, fmt, ...) APP_DEBUG(tag, DEBUG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif /* __APP_DEBUG_H__ */
