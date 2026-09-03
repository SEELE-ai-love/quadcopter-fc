#ifndef FC_CONTROLLER_H
#define FC_CONTROLLER_H

#include "fc_mixer.h"
#include "fc_pid.h"
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inputs to the attitude controller. */
typedef struct {
    fc_euler_t target;   /* desired roll/pitch/yaw, rad */
    fc_euler_t measured; /* estimated roll/pitch/yaw, rad */
    fc_vec3_t  rate;     /* measured body rates, rad/s */
} fc_attitude_input_t;

typedef struct {
    fc_pid_t rate_x;
    fc_pid_t rate_y;
    fc_pid_t rate_z;

    float angle_kp;      /* angle loop P gain */
    float max_rate;      /* rad/s */
} fc_controller_t;

void fc_controller_init(fc_controller_t *ctl);

/* Angle->rate->torque cascade. Returns control allocation request. */
fc_control_cmd_t fc_controller_update(fc_controller_t *ctl,
                                      const fc_attitude_input_t *in,
                                      float throttle,
                                      float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_CONTROLLER_H */
