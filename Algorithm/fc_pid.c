#include "fc_pid.h"

void fc_pid_reset(fc_pid_t *pid)
{
    if (pid == 0) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float fc_pid_update(fc_pid_t *pid, float error, float dt)
{
    if ((pid == 0) || (dt <= 0.0f)) {
        return 0.0f;
    }

    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->int_limit) {
        pid->integral = pid->int_limit;
    } else if (pid->integral < -pid->int_limit) {
        pid->integral = -pid->int_limit;
    }

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float out = pid->kp * error + pid->integral + pid->kd * derivative;

    /* Anti-windup: when the raw output saturates, pull the integral back so the
       unsaturated output stays inside the limit. */
    if (out > pid->out_limit) {
        float excess = out - pid->out_limit;
        pid->integral -= excess;
        out = pid->out_limit;
    } else if (out < -pid->out_limit) {
        float excess = out + pid->out_limit;
        pid->integral -= excess;
        out = -pid->out_limit;
    }

    return out;
}
