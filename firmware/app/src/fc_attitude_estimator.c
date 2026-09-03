#include "fc_attitude_estimator.h"
#include <math.h>

/* Basic complementary filter in Euler form. Intended as the S2 stage
   implementation; the production path can upgrade the core to quaternion
   integration without changing the module interface. */

static float fc_wrap_pi(float x)
{
    while (x > 3.14159265f) {
        x -= 6.28318530f;
    }
    while (x < -3.14159265f) {
        x += 6.28318530f;
    }
    return x;
}

void fc_attitude_estimator_init(fc_attitude_estimator_t *est)
{
    if (est == 0) {
        return;
    }
    est->gain = 0.02f;
    est->attitude.roll = 0.0f;
    est->attitude.pitch = 0.0f;
    est->attitude.yaw = 0.0f;
    est->initialized = 0;
}

void fc_attitude_estimator_update(fc_attitude_estimator_t *est,
                                  const fc_imu_sample_t *imu,
                                  float dt)
{
    if ((est == 0) || (imu == 0) || (dt <= 0.0f)) {
        return;
    }

    if (!est->initialized) {
        est->attitude.roll = atan2f(imu->ay, imu->az);
        est->attitude.pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay +
                                                     imu->az * imu->az));
        est->initialized = 1;
        return;
    }

    float roll = est->attitude.roll;
    float pitch = est->attitude.pitch;
    float yaw = est->attitude.yaw;

    float sin_r = sinf(roll);
    float cos_r = cosf(roll);
    float cos_p = cosf(pitch);

    /* Integrate gyro rates to Euler rates (Z-Y-X convention). */
    roll += (imu->gx + imu->gy * sin_r * tanf(pitch) +
             imu->gz * cos_r * tanf(pitch)) * dt;
    pitch += (imu->gy * cos_r - imu->gz * sin_r) * dt;
    if (fabsf(cos_p) > 0.01f) {
        yaw += (imu->gy * sin_r / cos_p + imu->gz * cos_r / cos_p) * dt;
    }

    /* Accel references only roll/pitch; yaw has no accel reference without a
       magnetometer. Filter gain and accelerometer validation belong in the
       hardware layer / tuning record. */
    float acc_roll = atan2f(imu->ay, imu->az);
    float acc_pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay +
                                             imu->az * imu->az));

    roll = roll * (1.0f - est->gain) + acc_roll * est->gain;
    pitch = pitch * (1.0f - est->gain) + acc_pitch * est->gain;

    est->attitude.roll = fc_wrap_pi(roll);
    est->attitude.pitch = pitch;
    est->attitude.yaw = fc_wrap_pi(yaw);
}
