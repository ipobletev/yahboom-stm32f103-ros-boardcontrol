#include "imu_app.h"
#include "icm20948.h"
#include "ak09916.h"
#include <string.h>

static axises_t current_accel;
static axises_t current_gyro;
static axises_t current_mag;
static imu_data_t current_imu_data;

void imu_init(bool debug) {
    ICM20948_init(debug);
    AK09916_init(debug);
}

void imu_update(void) {
    ICM20948_accel_read_g(&current_accel);
    ICM20948_gyro_read_dps(&current_gyro);
    
    current_imu_data.acc[0] = current_accel.x;
    current_imu_data.acc[1] = current_accel.y;
    current_imu_data.acc[2] = current_accel.z;
    
    current_imu_data.gyro[0] = current_gyro.x;
    current_imu_data.gyro[1] = current_gyro.y;
    current_imu_data.gyro[2] = current_gyro.z;

    AK09916_mag_read_uT(&current_mag);
    current_imu_data.mag[0] = current_mag.x;
    current_imu_data.mag[1] = current_mag.y;
    current_imu_data.mag[2] = current_mag.z;
}

void imu_get_data(imu_data_t *data) {
    if (data) {
        memcpy(data, &current_imu_data, sizeof(imu_data_t));
    }
}
