/**
 * @file app_debug.c
 * @brief Implementation of the debug printing utility.
 */

#include "app_debug.h"
#include <stdarg.h>

#if (APP_DEBUG_ENABLED == 1)
static const char* level_strings[] = {
    "Info",
    "Warn",
    "Error"
};
#endif

void app_debug_print(const char* tag, debug_level_t level, const char* fmt, ...) {
    #if (APP_DEBUG_ENABLED == 1)
    va_list args;
    
    // Print prefix: [TAG][LEVEL] 
    printf("[%s][%s] ", tag, level_strings[level]);
    
    // Print the user content
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    
    // Append newline for convenience (optional, depending on preference)
    printf("\r\n");
    #endif
}
