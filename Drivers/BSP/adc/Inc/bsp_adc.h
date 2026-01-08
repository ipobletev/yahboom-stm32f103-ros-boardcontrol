#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Initialize the ADC for battery voltage sensing.
 * @return true if success, false otherwise.
 */
bool bsp_adc_battery_init(void);

/**
 * @brief  Read the battery voltage.
 * @return The battery voltage in Volts.
 */
float bsp_adc_battery_read(void);

/**
 * @brief  Read the internal temperature.
 * @return The temperature in degC.
 */
float bsp_adc_temperature_read(void);

#endif /* BSP_ADC_H */
