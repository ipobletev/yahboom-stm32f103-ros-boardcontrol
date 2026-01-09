#include "../Inc/storage.h"
#include <stdbool.h>
#include <string.h>
#include "bsp_storage.h"
#include "app_debug.h"
#include "config.h"

#ifndef FLASH_STORAGE_ADDR
#define FLASH_STORAGE_ADDR    0x0803F800
#endif

static uint32_t calculate_checksum(const app_config_t *config) {
    uint32_t checksum = 0;
    const uint32_t *p = (const uint32_t*)config;
    // Calculate checksum excluding the checksum field itself
    for (size_t i = 0; i < (sizeof(app_config_t) / 4) - 1; i++) {
        checksum ^= p[i];
    }
    return checksum;
}

bool storage_load(app_config_t *config) {
    if (!BSP_Storage_Read(FLASH_STORAGE_ADDR, config, sizeof(app_config_t))) {
        return false;
    }
    
    if (config->magic != STORAGE_MAGIC) {
        APP_DEBUG_INFO("STORAGE", "Invalid magic number in storage (0x%08lX)", config->magic);
        return false;
    }
    
    uint32_t sum = calculate_checksum(config);
    if (sum != config->checksum) {
        APP_DEBUG_INFO("STORAGE", "Checksum mismatch: calc=0x%08lX, stored=0x%08lX", sum, config->checksum);
        return false;
    }
    
    return true;
}

bool storage_save(const app_config_t *config) {
    app_config_t copy = *config;
    copy.magic = STORAGE_MAGIC;
    copy.checksum = calculate_checksum(&copy);

    if (!BSP_Storage_Erase(FLASH_STORAGE_ADDR)) {
        APP_DEBUG_ERROR("STORAGE", "Flash erase failed via BSP!");
        return false;
    }

    if (!BSP_Storage_Write(FLASH_STORAGE_ADDR, &copy, sizeof(app_config_t))) {
        APP_DEBUG_ERROR("STORAGE", "Flash program failed via BSP!");
        return false;
    }

    APP_DEBUG_INFO("STORAGE", "Configuration saved to flash.");
    return true;
}
