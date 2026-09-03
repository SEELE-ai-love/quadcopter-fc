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
    float q[4];        /* attitude quaternion, w,x,y,z */
    float P[16];       /* 4x4 quaternion covariance, row-major */
    float gyro_noise;  /* gyro angle-noise density, rad/s/sqrt(Hz) */
    float accel_noise; /* accelerometer measurement sigma, normalized 1g units */
    fc_euler_t attitude; /* roll/pitch/yaw in rad */
    int initialized;
} fc_attitude_estimator_t;

void fc_attitude_estimator_init(fc_attitude_estimator_t *est);

/* Quaternion-state Kalman update:
   gyro propagation -> accelerometer measurement update. */
void fc_attitude_estimator_update(fc_attitude_estimator_t *est,
                                  const fc_imu_sample_t *imu,
                                  float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_ATTITUDE_ESTIMATOR_H */
