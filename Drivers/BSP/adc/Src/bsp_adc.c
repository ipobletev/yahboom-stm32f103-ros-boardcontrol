#include "bsp_adc.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc1;

bool bsp_adc_battery_init(void) {
    // Already initialized by MX_ADC1_Init() in main.c
    return true;
}

float bsp_adc_battery_read(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t raw_value = HAL_ADC_GetValue(&hadc1);
        
        // Voltage divider: R24=10k, R29=3.3k
        // V_adc = V_bat * (R29 / (R24 + R29))
        // V_bat = V_adc * (R24 + R29) / R29
        // V_adc = raw_value * 3.3 / 4095
        
        float v_adc = ((float)raw_value * 3.3f) / 4095.0f;
        float v_bat = v_adc * (10.0f + 3.3f) / 3.3f;
        
        return v_bat;
    }
    return 0.0f;
}

float bsp_adc_temperature_read(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Temp sensor is internal to ADC1, needs to be enabled in CR2
    // HAL_ADC_Init usually doesn't enable it unless requested in CubeMX
    ADC1->CR2 |= ADC_CR2_TSVREFE;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t raw_value = HAL_ADC_GetValue(&hadc1);
        
        // V_sense = raw_value * 3.3 / 4095
        // T (degC) = ((V_25 - V_sense) / Avg_Slope) + 25
        // Reference values for STM32F1: V_25 = 1.43V, Avg_Slope = 4.3mV/C
        
        float v_sense = ((float)raw_value * 3.3f) / 4095.0f;
        float temperature = ((1.43f - v_sense) / 0.0043f) + 25.0f;
        
        return temperature;
    }
    return 0.0f;
}
