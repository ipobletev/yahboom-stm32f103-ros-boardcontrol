#ifndef AK09916_H
#define AK09916_H

#include "icm20948.h"
#include <stdbool.h>

/* Magnetometer (AK09916) I2C address */
#define AK09916_ADDR 0x0C
#define MAG_WIA2 0x01
#define MAG_ST1  0x10
#define MAG_HXL  0x11
#define MAG_ST2  0x18
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32

/* Function Prototypes */
void AK09916_init(bool debug);
bool AK09916_mag_read(raw_data_t* data);
bool AK09916_mag_read_uT(axises_t* data);
bool AK09916_who_am_i(void);

#endif /* AK09916_H */
