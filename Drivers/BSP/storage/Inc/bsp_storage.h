#ifndef BSP_STORAGE_H
#define BSP_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize Storage (Flash) hardware.
 */
void BSP_Storage_Init(void);

/**
 * @brief Erase a page of Flash memory.
 * @param address Starting address of the page.
 * @return true if successful, false otherwise.
 */
bool BSP_Storage_Erase(uint32_t address);

/**
 * @brief Write data to Flash memory.
 * @param address Target address in Flash.
 * @param data Pointer to the data to write.
 * @param size Size of data in bytes (must be multiple of 4 for word programming).
 * @return true if successful, false otherwise.
 */
bool BSP_Storage_Write(uint32_t address, const void *data, uint32_t size);

/**
 * @brief Read data from Flash memory.
 * @param address Source address in Flash.
 * @param buffer Pointer to the buffer to store data.
 * @param size Size of data in bytes.
 * @return true if successful, false otherwise.
 */
bool BSP_Storage_Read(uint32_t address, void *buffer, uint32_t size);

#endif /* BSP_STORAGE_H */
