#ifndef ICM20948_H
#define ICM20948_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* SPI Configuration */
extern SPI_HandleTypeDef hspi2;
#define ICM20948_SPI &hspi2

/* Constants */
#define READ  0x80
#define WRITE 0x00

/* Data Types */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} axises_t;

typedef enum {
    ub_0 = 0,
    ub_1,
    ub_2,
    ub_3
} userbank_t;

typedef enum {
    _250dps = 0,
    _500dps,
    _1000dps,
    _2000dps
} gyro_full_scale_t;

typedef enum {
    _2g = 0,
    _4g,
    _8g,
    _16g
} accel_full_scale_t;

/* Register Defines */
#define REG_BANK_SEL 0x7F

// User Bank 0
#define B0_WHO_AM_I     0x00
#define B0_PWR_MGMT_1   0x06
#define B0_PWR_MGMT_2   0x07
#define B0_INT_PIN_CFG  0x0F
#define B0_USER_CTRL    0x03
#define B0_FIFO_EN_1    0x66
#define B0_FIFO_EN_2    0x67
#define B0_ACCEL_XOUT_H 0x2D
#define B0_GYRO_XOUT_H  0x33
#define B0_EXT_SLV_SENS_DATA_00 0x3B

// User Bank 2
#define B2_GYRO_SMPLRT_DIV 0x00
#define B2_GYRO_CONFIG_1   0x01
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG    0x14
#define B2_ODR_ALIGN_EN    0x09

// User Bank 3
#define B3_I2C_MST_CTRL    0x01
#define B3_I2C_MST_DELAY_CTRL 0x02
#define B3_I2C_SLV0_ADDR   0x03
#define B3_I2C_SLV0_REG    0x04
#define B3_I2C_SLV0_CTRL   0x05
#define B3_I2C_SLV0_DO     0x06
#define B3_I2C_SLV1_ADDR   0x07
#define B3_I2C_SLV1_REG    0x08
#define B3_I2C_SLV1_CTRL   0x09
#define B3_I2C_SLV1_DO     0x0A

/* Function Prototypes */
void ICM20948_init(bool debug);
void ICM20948_gyro_read(raw_data_t* data);
void ICM20948_accel_read(raw_data_t* data);
void ICM20948_gyro_read_dps(axises_t* data);
void ICM20948_accel_read_g(axises_t* data);
bool ICM20948_who_am_i(void);

/* Low Level Exports for Magnetometer */
uint8_t ICM20948_read_single_reg(userbank_t ub, uint8_t reg);
uint8_t* ICM20948_read_multiple_reg(userbank_t ub, uint8_t reg, uint8_t len);
void ICM20948_write_single_reg(userbank_t ub, uint8_t reg, uint8_t val);

#endif /* ICM20948_H */
