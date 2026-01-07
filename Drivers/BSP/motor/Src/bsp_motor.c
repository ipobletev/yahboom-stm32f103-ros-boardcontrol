#include "bsp_motor.h"
#include "tim.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// Note: TIM1 and TIM8 are used for PWM
#include <stdint.h>

void BSP_Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void BSP_Motor_Stop(uint8_t brake) {
    if (brake != 0) brake = 1;
    PWM_M1_A = brake * MOTOR_MAX_PULSE;
    PWM_M1_B = brake * MOTOR_MAX_PULSE;
    PWM_M2_A = brake * MOTOR_MAX_PULSE;
    PWM_M2_B = brake * MOTOR_MAX_PULSE;
    PWM_M3_A = brake * MOTOR_MAX_PULSE;
    PWM_M3_B = brake * MOTOR_MAX_PULSE;
    PWM_M4_A = brake * MOTOR_MAX_PULSE;
    PWM_M4_B = brake * MOTOR_MAX_PULSE;
}

void BSP_Motor_Set_Pwm(uint8_t id, int16_t pulse) {
    switch (id) {
        case BSP_MOTOR_ID_M1:
            {
                // Phase inversion according to motor mounting
                pulse = -pulse;
                
                if (pulse >= 0)
                {
                    PWM_M1_A = pulse;
                    PWM_M1_B = 0;
                }
                else
                {
                    PWM_M1_A = 0;
                    PWM_M1_B = -pulse;
                }
                break;
            }
        case BSP_MOTOR_ID_M2:
            {
                if (pulse >= 0)
                {
                    PWM_M2_A = pulse;
                    PWM_M2_B = 0;
                }
                else
                {
                    PWM_M2_A = 0;
                    PWM_M2_B = -pulse;
                }
                break;
            }
        case BSP_MOTOR_ID_M3:
            {
                if (pulse >= 0)
                {
                    PWM_M3_A = pulse;
                    PWM_M3_B = 0;
                }
                else
                {
                    PWM_M3_A = 0;
                    PWM_M3_B = -pulse;
                }
                break;
            }
        case BSP_MOTOR_ID_M4:
            {
                if (pulse >= 0)
                {
                    PWM_M4_A = pulse;
                    PWM_M4_B = 0;
                }
                else
                {
                    PWM_M4_A = 0;
                    PWM_M4_B = -pulse;
                }
                break;
            }
        default:
            break;
    }
}

int32_t BSP_Motor_Get_Encoder(uint8_t id) {
    switch (id) {
        case BSP_MOTOR_ID_M1: return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        case BSP_MOTOR_ID_M2: return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        case BSP_MOTOR_ID_M3: return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        case BSP_MOTOR_ID_M4: return (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        default: return 0;
    }
}

void BSP_Motor_Reset_Encoder(uint8_t id) {
    switch (id) {
        case BSP_MOTOR_ID_M1: __HAL_TIM_SET_COUNTER(&htim2, 0); break;
        case BSP_MOTOR_ID_M2: __HAL_TIM_SET_COUNTER(&htim3, 0); break;
        case BSP_MOTOR_ID_M3: __HAL_TIM_SET_COUNTER(&htim4, 0); break;
        case BSP_MOTOR_ID_M4: __HAL_TIM_SET_COUNTER(&htim5, 0); break;
    }
}
