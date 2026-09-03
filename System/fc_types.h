#ifndef FC_TYPES_H
#define FC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;
    float pitch;
    float yaw;
} fc_euler_t;

typedef struct {
    float x;
    float y;
    float z;
} fc_vec3_t;

typedef enum {
    FC_STATE_DISARMED = 0,
    FC_STATE_ARMED,
    FC_STATE_LANDING,
    FC_STATE_FAILSAFE,
    FC_STATE_EMERGENCY
} fc_state_t;

typedef struct {
    uint32_t loop_count;
    uint32_t fault_bits;   /* bit0 rc lost, bit1 low voltage, bit2 imu bad */
} fc_status_t;

#ifdef __cplusplus
}
#endif

#endif /* FC_TYPES_H */
