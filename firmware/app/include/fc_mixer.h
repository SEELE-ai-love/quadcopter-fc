#ifndef FC_MIXER_H
#define FC_MIXER_H

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* throttle: 0..1, commands: -1..1. Output motor 0..1. */
typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} fc_control_cmd_t;

void fc_mixer_quad_x(const fc_control_cmd_t *cmd, float motor[4]);

#ifdef __cplusplus
}
#endif

#endif /* FC_MIXER_H */
