#ifndef FC_ATTITUDE_ESTIMATOR_H
#define FC_ATTITUDE_ESTIMATOR_H

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax;   /* m/s^2 */
    float ay;
    float az;
    float gx;   /* rad/s */
    float gy;
    float gz;
} fc_imu_sample_t;

typedef struct {
    float gain;         /* accel correction gain, e.g. 0.02 per update */
    fc_euler_t attitude; /* roll/pitch/yaw in rad */
    int initialized;
} fc_attitude_estimator_t;

void fc_attitude_estimator_init(fc_attitude_estimator_t *est);
void fc_attitude_estimator_update(fc_attitude_estimator_t *est,
                                  const fc_imu_sample_t *imu,
                                  float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_ATTITUDE_ESTIMATOR_H */
