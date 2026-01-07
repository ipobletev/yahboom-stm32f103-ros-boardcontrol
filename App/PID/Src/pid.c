#include "pid.h"
#include <string.h>

void pid_init(pid_control_t *pid, float kp, float ki, float kd, float max_out, float min_out) {
    memset(pid, 0, sizeof(pid_control_t));
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_output = max_out;
    pid->min_output = min_out;
}

void pid_reset(pid_control_t *pid) {
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->output = 0;
    pid->target = 0;
    pid->current = 0;
}

float pid_calculate(pid_control_t *pid, float target, float current) {
    pid->target = target;
    pid->current = current;
    
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target - current;
    
    // Incremental PID formula:
    // ΔU = Kp * (e[k] - e[k-1]) + Ki * e[k] + Kd * (e[k] - 2*e[k-1] + e[k-2])
    float delta_u = pid->Kp * (pid->error[0] - pid->error[1]) +
                    pid->Ki * pid->error[0] +
                    pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    
    pid->output += delta_u;
    
    // Limit output
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    else if (pid->output < pid->min_output) pid->output = pid->min_output;
    
    return pid->output;
}
