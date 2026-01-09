#include "bsp_storage.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include <string.h>

void BSP_Storage_Init(void) {
    // HAL Flash initialization is usually handled by HAL_Init
}

bool BSP_Storage_Erase(uint32_t address) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages = 1;

    uint32_t pageError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &pageError);
    
    HAL_FLASH_Lock();

    return (status == HAL_OK);
}

bool BSP_Storage_Write(uint32_t address, const void *data, uint32_t size) {
    HAL_FLASH_Unlock();

    bool success = true;
    uint32_t *pData = (uint32_t*)data;
    
    // Size must be in bytes, word is 4 bytes
    for (uint32_t i = 0; i < size; i += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, *pData++) != HAL_OK) {
            success = false;
            break;
        }
    }

    HAL_FLASH_Lock();
    return success;
}

bool BSP_Storage_Read(uint32_t address, void *buffer, uint32_t size) {
    // On STM32, Flash is memory-mapped, so we can use memcpy
    memcpy(buffer, (void*)address, size);
    return true;
}
