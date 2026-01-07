#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>
#include "main.h"

// Hardware-specific PWM access
#define PWM_M1_A TIM8->CCR1
#define PWM_M1_B TIM8->CCR2
#define PWM_M2_A TIM8->CCR3
#define PWM_M2_B TIM8->CCR4
#define PWM_M3_A TIM1->CCR4
#define PWM_M3_B TIM1->CCR1
#define PWM_M4_A TIM1->CCR2
#define PWM_M4_B TIM1->CCR3

#define MOTOR_MAX_PULSE 3600

typedef enum {
    BSP_MOTOR_ID_M1,
    BSP_MOTOR_ID_M2,
    BSP_MOTOR_ID_M3,
    BSP_MOTOR_ID_M4
} bsp_motor_id_t;

/**
 * @brief Initialize hardware timers for PWM.
 */
void BSP_Motor_Init(void);

/**
 * @brief Update hardware PWM registers.
 * @param id Motor hardware ID.
 * @param pulse PWM pulse value.
 */
void BSP_Motor_Set_Pwm(uint8_t id, int16_t pulse);

/**
 * @brief Stop motors at hardware level.
 * @param brake Apply braking via hardware PWM if non-zero.
 */
void BSP_Motor_Stop(uint8_t brake);

/**
 * @brief Get encoder count for a specific motor.
 * @param id Motor hardware ID.
 * @return Encoder count.
 */
int32_t BSP_Motor_Get_Encoder(uint8_t id);

/**
 * @brief Reset encoder count for a specific motor.
 * @param id Motor hardware ID.
 */
void BSP_Motor_Reset_Encoder(uint8_t id);

#endif /* BSP_MOTOR_H */
