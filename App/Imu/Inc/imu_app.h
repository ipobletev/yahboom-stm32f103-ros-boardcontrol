#ifndef IMU_APP_H
#define IMU_APP_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float acc[3];
    float gyro[3];
    float mag[3];
} imu_data_t;

/**
 * @brief Initialize IMU application.
 */
bool imu_init(uint8_t retries, bool debug);

/**
 * @brief Update IMU data.
 */
void imu_update(void);

/**
 * @brief Get latest IMU data.
 * @param data Pointer to imu_data_t structure.
 */
void imu_get_data(imu_data_t *data);

/**
 * @brief Check IMU sensors health (Who Am I registers).
 * @return true if all sensors are OK, false otherwise.
 */
bool imu_health_check(void);

#endif /* IMU_APP_H */
