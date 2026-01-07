#include "icm20948.h"
#include "app_debug.h"
#include <stdio.h>

static raw_data_t g_raw_accel;
static raw_data_t g_raw_gyro;

static float g_scale_accel = 2048.0; 
static float g_scale_gyro = 16.4;    

static userbank_t g_current_bank = ub_3; 
static bool g_debug = false;

/* SPI Helper Functions */
static void ICM20948_NoActive(void) {
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);
}

static void ICM20948_Active(void) {
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_RESET);
}

static void select_user_bank(userbank_t ub) {
    if (ub != g_current_bank) {
        uint8_t write_reg[2];
        write_reg[0] = WRITE | REG_BANK_SEL;
        write_reg[1] = (uint8_t)(ub << 4);
        ICM20948_Active();
        HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
        ICM20948_NoActive();
        g_current_bank = ub;
    }
}

uint8_t ICM20948_read_single_reg(userbank_t ub, uint8_t reg) {
    uint8_t read_reg = READ | reg;
    uint8_t reg_val;
    select_user_bank(ub);
    ICM20948_Active();
    HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
    HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
    ICM20948_NoActive();
    return reg_val;
}

uint8_t* ICM20948_read_multiple_reg(userbank_t ub, uint8_t reg, uint8_t len) {
    uint8_t read_reg = READ | reg;
    static uint8_t reg_val[10]; 
    select_user_bank(ub);
    ICM20948_Active();
    HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
    HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
    ICM20948_NoActive();
    return reg_val;
}

void ICM20948_write_single_reg(userbank_t ub, uint8_t reg, uint8_t val) {
    uint8_t write_reg[2];
    write_reg[0] = WRITE | reg;
    write_reg[1] = val;
    select_user_bank(ub);
    ICM20948_Active();
    HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
    ICM20948_NoActive();
}

/* Initialization Functions */

static void ICM20948_device_reset(void) {
    ICM20948_write_single_reg(ub_0, B0_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
}

static void ICM20948_wakeup(void) {
    ICM20948_write_single_reg(ub_0, B0_PWR_MGMT_1, 0x01);
}

static void ICM20948_clock_source(uint8_t src) {
    uint8_t val = ICM20948_read_single_reg(ub_0, B0_PWR_MGMT_1);
    val &= 0xF8;
    val |= (src & 0x07);
    ICM20948_write_single_reg(ub_0, B0_PWR_MGMT_1, val);
}

static void ICM20948_odr_align_enable(void) {
    // Basic implementation
}

static void ICM20948_spi_slave_enable(void) {
    uint8_t val = ICM20948_read_single_reg(ub_0, B0_USER_CTRL);
    val |= 0x10; // I2C_IF_DIS
    ICM20948_write_single_reg(ub_0, B0_USER_CTRL, val);
}

static void ICM20948_gyro_sample_rate_divider(uint8_t d) {
    ICM20948_write_single_reg(ub_2, B2_GYRO_SMPLRT_DIV, d);
}

static void ICM20948_accel_sample_rate_divider(uint8_t d) {
    ICM20948_write_single_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, 0);
    ICM20948_write_single_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, d);
}

static void ICM20948_gyro_full_scale_select(gyro_full_scale_t fs) {
    uint8_t val = ICM20948_read_single_reg(ub_2, B2_GYRO_CONFIG_1);
    val &= 0xF9;
    val |= (fs << 1);
    ICM20948_write_single_reg(ub_2, B2_GYRO_CONFIG_1, val);
    
    switch(fs) {
        case _250dps: g_scale_gyro = 131.0; break;
        case _500dps: g_scale_gyro = 65.5; break;
        case _1000dps: g_scale_gyro = 32.8; break;
        case _2000dps: g_scale_gyro = 16.4; break;
    }
}

static void ICM20948_accel_full_scale_select(accel_full_scale_t fs) {
    uint8_t val = ICM20948_read_single_reg(ub_2, B2_ACCEL_CONFIG);
    val &= 0xF9;
    val |= (fs << 1);
    ICM20948_write_single_reg(ub_2, B2_ACCEL_CONFIG, val);

    switch(fs) {
        case _2g: g_scale_accel = 16384.0; break;
        case _4g: g_scale_accel = 8192.0; break;
        case _8g: g_scale_accel = 4096.0; break;
        case _16g: g_scale_accel = 2048.0; break;
    }
}

void ICM20948_init(bool debug) {
    g_debug = debug;
    ICM20948_device_reset();
    HAL_Delay(100);
    g_current_bank = ub_0; 
    
    if (g_debug) APP_DEBUG_INFO("ICM20948", "Checking ICM20948 WHO_AM_I...\r\n");
    uint8_t id = ICM20948_read_single_reg(ub_0, B0_WHO_AM_I);
    if (id != 0xEA) {
        if (g_debug) APP_DEBUG_ERROR("ICM20948", "WHO_AM_I failed: 0x%02X (expected 0xEA)\r\n", id);
    } else {
        if (g_debug) APP_DEBUG_INFO("ICM20948", "detected!\r\n");
    }

    ICM20948_wakeup();
    ICM20948_clock_source(1);
    ICM20948_odr_align_enable();
    ICM20948_spi_slave_enable();
    
    ICM20948_gyro_sample_rate_divider(0);
    ICM20948_accel_sample_rate_divider(0);

    ICM20948_gyro_full_scale_select(_2000dps);
    ICM20948_accel_full_scale_select(_16g);

    ICM20948_write_single_reg(ub_0, B0_PWR_MGMT_2, 0x00);
}

bool ICM20948_who_am_i(void) {
    uint8_t id = ICM20948_read_single_reg(ub_0, B0_WHO_AM_I);
    return (id == 0xEA);
}

void ICM20948_accel_read(raw_data_t* data) {
    uint8_t* temp = ICM20948_read_multiple_reg(ub_0, B0_ACCEL_XOUT_H, 6);
    data->x = (int16_t)(temp[0] << 8 | temp[1]);
    data->y = (int16_t)(temp[2] << 8 | temp[3]);
    data->z = (int16_t)(temp[4] << 8 | temp[5]) + (int16_t)g_scale_accel; 
}

void ICM20948_accel_read_g(axises_t* data) {
    ICM20948_accel_read(&g_raw_accel);
    data->x = (float)(g_raw_accel.x / g_scale_accel);
    data->y = (float)(g_raw_accel.y / g_scale_accel);
    data->z = (float)(g_raw_accel.z / g_scale_accel);
}

void ICM20948_gyro_read(raw_data_t* data) {
    uint8_t* temp = ICM20948_read_multiple_reg(ub_0, B0_GYRO_XOUT_H, 6);
    data->x = (int16_t)(temp[0] << 8 | temp[1]);
    data->y = (int16_t)(temp[2] << 8 | temp[3]);
    data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void ICM20948_gyro_read_dps(axises_t* data) {
    ICM20948_gyro_read(&g_raw_gyro);
    data->x = (float)(g_raw_gyro.x / g_scale_gyro);
    data->y = (float)(g_raw_gyro.y / g_scale_gyro);
    data->z = (float)(g_raw_gyro.z / g_scale_gyro);
}
