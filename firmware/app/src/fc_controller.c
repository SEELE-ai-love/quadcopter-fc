#include "fc_controller.h"
#include <math.h>

static float wrap_pi(float x)
{
    while (x > 3.14159265f) x -= 6.28318530f;
    while (x < -3.14159265f) x += 6.28318530f;
    return x;
}

static void pid_config(fc_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_limit = 1.0f;
    pid->int_limit = 0.15f;
    fc_pid_reset(pid);
}

void fc_controller_init(fc_controller_t *ctl)
{
    if (ctl == 0) {
        return;
    }
    ctl->angle_kp = 4.5f;
    ctl->max_rate = 3.0f;
    pid_config(&ctl->rate_x, 0.25f, 0.45f, 0.004f);
    pid_config(&ctl->rate_y, 0.25f, 0.45f, 0.004f);
    pid_config(&ctl->rate_z, 0.20f, 0.35f, 0.003f);
}

fc_control_cmd_t fc_controller_update(fc_controller_t *ctl,
                                      const fc_attitude_input_t *in,
                                      float throttle,
                                      float dt)
{
    fc_control_cmd_t cmd = {throttle, 0.0f, 0.0f, 0.0f};
    if ((ctl == 0) || (in == 0) || (dt <= 0.0f)) {
        return cmd;
    }

    /* Outer angle loop: P controller producing a rate reference. */
    float roll_err  = wrap_pi(in->target.roll  - in->measured.roll);
    float pitch_err = wrap_pi(in->target.pitch - in->measured.pitch);
    float yaw_err   = wrap_pi(in->target.yaw   - in->measured.yaw);

    float rate_ref_x = ctl->angle_kp * roll_err;
    float rate_ref_y = ctl->angle_kp * pitch_err;
    float rate_ref_z = ctl->angle_kp * yaw_err;

    if (rate_ref_x >  ctl->max_rate) rate_ref_x =  ctl->max_rate;
    if (rate_ref_x < -ctl->max_rate) rate_ref_x = -ctl->max_rate;
    if (rate_ref_y >  ctl->max_rate) rate_ref_y =  ctl->max_rate;
    if (rate_ref_y < -ctl->max_rate) rate_ref_y = -ctl->max_rate;
    if (rate_ref_z >  ctl->max_rate) rate_ref_z =  ctl->max_rate;
    if (rate_ref_z < -ctl->max_rate) rate_ref_z = -ctl->max_rate;

    /* Inner rate loop: torque axes expressed as normalized mixer axes. */
    cmd.roll  = fc_pid_update(&ctl->rate_x, rate_ref_x - in->rate.x, dt);
    cmd.pitch = fc_pid_update(&ctl->rate_y, rate_ref_y - in->rate.y, dt);
    cmd.yaw   = fc_pid_update(&ctl->rate_z, rate_ref_z - in->rate.z, dt);

    return cmd;
}
