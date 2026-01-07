#include "ak09916.h"
#include "app_debug.h"
#include <stdio.h>

static raw_data_t g_raw_mag;
static bool g_debug = false;

static void AK09916_write_reg(uint8_t reg, uint8_t data) {
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_ADDR, AK09916_ADDR); 
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_DO, data);
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81); 
}

static uint8_t AK09916_read_reg(uint8_t reg) {
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_ADDR, AK09916_ADDR | 0x80); 
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81); 
    HAL_Delay(10);
    return ICM20948_read_single_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static uint8_t* AK09916_read_multiple(uint8_t reg, uint8_t len) {
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_ADDR, AK09916_ADDR | 0x80); 
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_REG, reg);
    ICM20948_write_single_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len); 
    HAL_Delay(10);
    return ICM20948_read_multiple_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}

static void ICM20948_i2c_master_reset(void) {
    uint8_t val = ICM20948_read_single_reg(ub_0, B0_USER_CTRL);
    val |= 0x02; 
    ICM20948_write_single_reg(ub_0, B0_USER_CTRL, val);
    HAL_Delay(10);
}

static void ICM20948_i2c_master_init(void) {
    uint8_t val = ICM20948_read_single_reg(ub_0, B0_USER_CTRL);
    val |= 0x30; 
    ICM20948_write_single_reg(ub_0, B0_USER_CTRL, val);
    ICM20948_write_single_reg(ub_3, B3_I2C_MST_CTRL, 0x17); 
}

void AK09916_init(bool debug) {
    g_debug = debug;
    ICM20948_i2c_master_reset();
    ICM20948_i2c_master_init();

    while(AK09916_read_reg(MAG_WIA2) != 0x09) {
        if (g_debug) APP_DEBUG_INFO("AK09916", "Waiting for AK09916...\r\n");
        HAL_Delay(100);
    }
    if (g_debug) APP_DEBUG_INFO("AK09916", "Magnetometer detected!\r\n");

    AK09916_write_reg(MAG_CNTL3, 0x01);
    HAL_Delay(100);
    AK09916_write_reg(MAG_CNTL2, 0x08);
}

bool AK09916_mag_read(raw_data_t* data) {
    uint8_t drdy = AK09916_read_reg(MAG_ST1) & 0x01;
    if (!drdy) return false;

    uint8_t* temp = AK09916_read_multiple(MAG_HXL, 6);

    uint8_t hofl = AK09916_read_reg(MAG_ST2) & 0x08;
    if (hofl) return false;

    data->x = (int16_t)(temp[1] << 8 | temp[0]);
    data->y = (int16_t)(temp[3] << 8 | temp[2]);
    data->z = (int16_t)(temp[5] << 8 | temp[4]);
    
    return true;
}

bool AK09916_mag_read_uT(axises_t* data) {
    AK09916_mag_read(&g_raw_mag);
    data->x = (float)(g_raw_mag.x * 0.15f);
    data->y = (float)(g_raw_mag.y * 0.15f);
    data->z = (float)(g_raw_mag.z * 0.15f);
    return true;
}

bool AK09916_who_am_i(void) {
    return (AK09916_read_reg(MAG_WIA2) == 0x09);
}
