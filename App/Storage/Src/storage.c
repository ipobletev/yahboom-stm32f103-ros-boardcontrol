#include "../Inc/storage.h"
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "app_debug.h"

// STM32F103RCTx has 256KB Flash. Page size is 2KB.
// Last page starts at 0x0803F800
#define FLASH_STORAGE_ADDR 0x0803F800
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE    2048
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
    memcpy(config, (void*)FLASH_STORAGE_ADDR, sizeof(app_config_t));
    
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

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_STORAGE_ADDR;
    erase.NbPages = 1;

    uint32_t pageError = 0;
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        APP_DEBUG_ERROR("STORAGE", "Flash erase failed!");
        return false;
    }

    uint32_t *pData = (uint32_t*)&copy;
    for (size_t i = 0; i < sizeof(app_config_t); i += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + i, *pData++) != HAL_OK) {
            HAL_FLASH_Lock();
            APP_DEBUG_ERROR("STORAGE", "Flash program failed at offset %d", i);
            return false;
        }
    }

    HAL_FLASH_Lock();
    APP_DEBUG_INFO("STORAGE", "Configuration saved to flash.");
    return true;
}
