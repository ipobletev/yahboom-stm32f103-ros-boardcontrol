#include "imu_app.h"
#include "icm20948.h"
#include <stdint.h>
#include <string.h>

static axises current_accel;
static axises current_gyro;
static axises current_mag;
static imu_data_t current_imu_data;

bool imu_init(uint8_t retries, bool debug) {
    icm20948_init();
    ak09916_init();
    return true;
}

void imu_update(void) {
    icm20948_accel_read_g(&current_accel);
    icm20948_gyro_read_dps(&current_gyro);
    
    current_imu_data.acc[0] = current_accel.x;
    current_imu_data.acc[1] = current_accel.y;
    current_imu_data.acc[2] = current_accel.z;
    
    current_imu_data.gyro[0] = current_gyro.x;
    current_imu_data.gyro[1] = current_gyro.y;
    current_imu_data.gyro[2] = current_gyro.z;

    ak09916_mag_read_uT(&current_mag);
    current_imu_data.mag[0] = current_mag.x;
    current_imu_data.mag[1] = current_mag.y;
    current_imu_data.mag[2] = current_mag.z;
}

void imu_get_data(imu_data_t *data) {
    if (data) {
        memcpy(data, &current_imu_data, sizeof(imu_data_t));
    }
}

bool imu_health_check(void) {
    bool icm_ok = icm20948_who_am_i();
    bool ak_ok = ak09916_who_am_i();
    return icm_ok && ak_ok;
}
