#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float target;
    float current;
    
    float error[3]; // error[0]: e(k), error[1]: e(k-1), error[2]: e(k-2)
    
    float output;
    float max_output;
    float min_output;
} pid_control_t;

/**
 * @brief Initialize PID controller with gains and limits.
 */
void pid_init(pid_control_t *pid, float kp, float ki, float kd, float max_out, float min_out);

/**
 * @brief Reset PID errors and output.
 */
void pid_reset(pid_control_t *pid);

/**
 * @brief Calculate incremental PID output.
 * @return The new total output.
 */
float pid_calculate(pid_control_t *pid, float target, float current);

#endif /* PID_H */
