#ifndef FC_SAFETY_H
#define FC_SAFETY_H

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    fc_state_t state;
    int link_ok;
    int low_voltage;
    int imu_ok;
    int throttle_low;   /* 1 only when the pilot throttle is at min */
} fc_safety_t;

void  fc_safety_init(fc_safety_t *fs);
void  fc_safety_request_arm(fc_safety_t *fs);
void  fc_safety_request_disarm(fc_safety_t *fs);
void  fc_safety_update(fc_safety_t *fs);
const char *fc_safety_name(fc_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* FC_SAFETY_H */
