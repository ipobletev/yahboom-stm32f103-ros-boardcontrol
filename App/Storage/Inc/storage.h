#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "motor.h"

#define STORAGE_MAGIC 0x5A5A1234

typedef struct {
    uint32_t magic;
    motor_pid_params_t pid[4];
    float wheel_diameter;
    uint32_t checksum;
} app_config_t;

/**
 * @brief Initialize storage and load configuration.
 * @return True if valid config was loaded, false otherwise.
 */
bool storage_init(void);

/**
 * @brief Load configuration from Flash.
 * @param config Pointer to store the loaded config.
 * @return True if valid config was loaded.
 */
bool storage_load(app_config_t *config);

/**
 * @brief Save configuration to Flash.
 * @param config Pointer to the config to save.
 * @return True if successfully saved.
 */
bool storage_save(const app_config_t *config);

#endif // STORAGE_H
