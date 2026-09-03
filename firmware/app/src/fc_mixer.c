#include "fc_mixer.h"

/* Quad-X allocation. Motor numbering and signs MUST be verified on the real
   frame before any armed test: one wrong sign means an immediate flip. */
void fc_mixer_quad_x(const fc_control_cmd_t *cmd, float motor[4])
{
    if ((cmd == 0) || (motor == 0)) {
        return;
    }

    float throttle = cmd->throttle;
    float m0 = throttle + cmd->roll + cmd->pitch + cmd->yaw;
    float m1 = throttle - cmd->roll + cmd->pitch - cmd->yaw;
    float m2 = throttle + cmd->roll - cmd->pitch - cmd->yaw;
    float m3 = throttle - cmd->roll - cmd->pitch + cmd->yaw;

    /* Keep the output inside 0..1 without changing the ratio between motors. */
    float lo = m0;
    float hi = m0;
    float vals[4] = {m0, m1, m2, m3};
    for (int i = 1; i < 4; ++i) {
        if (vals[i] < lo) lo = vals[i];
        if (vals[i] > hi) hi = vals[i];
    }
    if (lo < 0.0f) {
        float shift = -lo;
        for (int i = 0; i < 4; ++i) {
            vals[i] += shift;
        }
        hi += shift;
    }
    if (hi > 1.0f) {
        float scale = 1.0f / hi;
        for (int i = 0; i < 4; ++i) {
            vals[i] *= scale;
        }
    }

    motor[0] = vals[0];
    motor[1] = vals[1];
    motor[2] = vals[2];
    motor[3] = vals[3];
}
