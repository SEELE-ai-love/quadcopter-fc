#include "fc_attitude_estimator.h"
#include <math.h>

#define FC_GYRO_NOISE_DEFAULT  0.0035f  /* rad/s / sqrt(Hz) */
#define FC_ACCEL_NOISE_DEFAULT 0.0200f  /* normalized 1g units */

static float fc_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] +
                    q[2] * q[2] + q[3] * q[3]);
    if (n > 1e-12f) {
        float inv = 1.0f / n;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
}

static void quat_to_euler(const float q[4], fc_euler_t *e)
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    e->roll  = atan2f(2.0f * (w * x + y * z),
                      1.0f - 2.0f * (x * x + y * y));
    e->pitch = asinf(fc_clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
    e->yaw   = atan2f(2.0f * (w * z + x * y),
                      1.0f - 2.0f * (y * y + z * z));
}

static void mat4_zero(float m[16])
{
    for (int i = 0; i < 16; ++i) m[i] = 0.0f;
}

static void mat4_mul(const float a[16], const float b[16], float out[16])
{
    float tmp[16];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a[4 * r + k] * b[4 * k + c];
            }
            tmp[4 * r + c] = sum;
        }
    }
    for (int i = 0; i < 16; ++i) out[i] = tmp[i];
}

static void mat4_transpose(const float m[16], float out[16])
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[4 * r + c] = m[4 * c + r];
        }
    }
}

static int mat3_inv(const float m[9], float out[9])
{
    float a = m[0], b = m[1], c = m[2];
    float d = m[3], e = m[4], f = m[5];
    float g = m[6], h = m[7], i = m[8];
    float A = e * i - f * h;
    float B = f * g - d * i;
    float C = d * h - e * g;
    float det = a * A + b * B + c * C;
    if (fabsf(det) < 1e-12f) {
        return -1;
    }
    float inv = 1.0f / det;
    out[0] = A * inv;
    out[1] = (c * h - b * i) * inv;
    out[2] = (b * f - c * e) * inv;
    out[3] = B * inv;
    out[4] = (a * i - c * g) * inv;
    out[5] = (c * d - a * f) * inv;
    out[6] = C * inv;
    out[7] = (b * g - a * h) * inv;
    out[8] = (a * e - b * d) * inv;
    return 0;
}

/* h(q) predicts the normalized gravity vector seen in body axes.
   H is the 3x4 Jacobian of h with respect to the quaternion. */
static void accel_model(const float q[4], float h[3], float H[12])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    h[0] = 2.0f * (x * z - w * y);
    h[1] = 2.0f * (y * z + w * x);
    h[2] = 1.0f - 2.0f * (x * x + y * y);

    H[0] = -2.0f * y;
    H[1] =  2.0f * z;
    H[2] = -2.0f * w;
    H[3] =  2.0f * x;

    H[4] =  2.0f * x;
    H[5] =  2.0f * w;
    H[6] =  2.0f * z;
    H[7] =  2.0f * y;

    H[8]  = 0.0f;
    H[9]  = -4.0f * x;
    H[10] = -4.0f * y;
    H[11] = 0.0f;
}

/* Quaternion update from the gyro rates. */
static void gyro_propagate(fc_attitude_estimator_t *est,
                           const float g[3], float dt)
{
    float m[16];
    float f[16];
    float fp[16];
    float qp[4];
    float tmp[16];

    mat4_zero(m);
    m[1] = -g[0];  m[2] = -g[1];  m[3] = -g[2];
    m[4] =  g[0];  m[6] =  g[2];  m[7] = -g[1];
    m[8] =  g[1];  m[9] = -g[2];  m[11] = g[0];
    m[12] = g[2];  m[13] = g[1];  m[14] = -g[0];

    qp[0] = est->q[0] + 0.5f * dt * (m[0] * est->q[0] + m[1] * est->q[1] +
                                     m[2] * est->q[2] + m[3] * est->q[3]);
    qp[1] = est->q[1] + 0.5f * dt * (m[4] * est->q[0] + m[5] * est->q[1] +
                                     m[6] * est->q[2] + m[7] * est->q[3]);
    qp[2] = est->q[2] + 0.5f * dt * (m[8] * est->q[0] + m[9] * est->q[1] +
                                     m[10] * est->q[2] + m[11] * est->q[3]);
    qp[3] = est->q[3] + 0.5f * dt * (m[12] * est->q[0] + m[13] * est->q[1] +
                                     m[14] * est->q[2] + m[15] * est->q[3]);
    quat_normalize(qp);

    /* Linearized propagation F = I + 0.5*dt*M, P = F*P*F' + Q. */
    for (int i = 0; i < 16; ++i) {
        f[i] = m[i] * (0.5f * dt);
    }
    f[0] += 1.0f;
    f[5] += 1.0f;
    f[10] += 1.0f;
    f[15] += 1.0f;

    mat4_mul(f, est->P, tmp);
    mat4_transpose(f, fp);
    mat4_mul(tmp, fp, est->P);

    /* Each gyro noise contribution maps into half of an angular error. */
    float qnoise = 0.25f * dt * est->gyro_noise * est->gyro_noise;
    est->P[0] += qnoise;
    est->P[5] += qnoise;
    est->P[10] += qnoise;
    est->P[15] += qnoise;

    est->q[0] = qp[0];
    est->q[1] = qp[1];
    est->q[2] = qp[2];
    est->q[3] = qp[3];
}

static void accel_update(fc_attitude_estimator_t *est, const float a[3])
{
    float h[3], H[12];
    float Ht[12];
    float PHt[12];
    float S[9], Sinv[9];
    float K[12];
    float kh[16], khp[16];
    float innov[3];

    accel_model(est->q, h, H);

    innov[0] = a[0] - h[0];
    innov[1] = a[1] - h[1];
    innov[2] = a[2] - h[2];
    float inorm = sqrtf(innov[0] * innov[0] +
                        innov[1] * innov[1] +
                        innov[2] * innov[2]);
    /* Dynamic acceleration changes both magnitude and direction; gate it out. */
    if (inorm > 0.35f) {
        return;
    }

    /* Ht = H^T (4x3), PHt = P * Ht (4x3). */
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            Ht[3 * c + r] = H[4 * r + c];
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += est->P[4 * r + k] * Ht[3 * k + c];
            }
            PHt[3 * r + c] = sum;
        }
    }

    /* S = H * PHt + R (3x3). */
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += H[4 * r + k] * PHt[3 * k + c];
            }
            S[3 * r + c] = sum;
        }
    }
    float rvar = est->accel_noise * est->accel_noise;
    S[0] += rvar;
    S[4] += rvar;
    S[8] += rvar;
    if (mat3_inv(S, Sinv) != 0) {
        return;
    }

    /* K = PHt * Sinv (4x3). */
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += PHt[3 * r + k] * Sinv[3 * k + c];
            }
            K[3 * r + c] = sum;
        }
    }

    /* q += K * innovation; renormalize after update. */
    float dq[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int r = 0; r < 4; ++r) {
        dq[r] = K[3 * r] * innov[0] +
                K[3 * r + 1] * innov[1] +
                K[3 * r + 2] * innov[2];
        est->q[r] += dq[r];
    }
    quat_normalize(est->q);

    /* P = (I - K*H) * P, then symmetrize. */
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += K[3 * r + k] * H[4 * k + c];
            }
            kh[4 * r + c] = sum;
        }
    }
    mat4_mul(kh, est->P, khp);
    for (int i = 0; i < 16; ++i) {
        est->P[i] -= khp[i];
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = r + 1; c < 4; ++c) {
            float avg = 0.5f * (est->P[4 * r + c] + est->P[4 * c + r]);
            est->P[4 * r + c] = avg;
            est->P[4 * c + r] = avg;
        }
    }
}

void fc_attitude_estimator_init(fc_attitude_estimator_t *est)
{
    if (est == 0) {
        return;
    }
    est->q[0] = 1.0f;
    est->q[1] = 0.0f;
    est->q[2] = 0.0f;
    est->q[3] = 0.0f;
    for (int i = 0; i < 16; ++i) {
        est->P[i] = 0.0f;
    }
    est->P[0] = 1e-3f;
    est->P[5] = 1e-3f;
    est->P[10] = 1e-3f;
    est->P[15] = 1e-3f;
    est->gyro_noise = FC_GYRO_NOISE_DEFAULT;
    est->accel_noise = FC_ACCEL_NOISE_DEFAULT;
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
        /* Level the quaternion from the gravity direction on first frame. */
        float roll = atan2f(imu->ay, imu->az);
        float pitch = atan2f(-imu->ax,
                             sqrtf(imu->ay * imu->ay + imu->az * imu->az));
        float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
        float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
        est->q[0] = cp * cr;
        est->q[1] = cp * sr;
        est->q[2] = sp * cr;
        est->q[3] = -sp * sr;
        quat_normalize(est->q);
        est->initialized = 1;
    }

    float g[3] = {imu->gx, imu->gy, imu->gz};
    gyro_propagate(est, g, dt);

    float anorm = sqrtf(imu->ax * imu->ax +
                        imu->ay * imu->ay +
                        imu->az * imu->az);
    /* Only stationary / low-acceleration samples enter the Kalman update. */
    if ((anorm > 0.8f) && (anorm < 1.25f)) {
        float a[3] = {imu->ax / anorm, imu->ay / anorm, imu->az / anorm};
        accel_update(est, a);
    }

    quat_to_euler(est->q, &est->attitude);
}
