#ifndef __BSP_IMU_EKF_H
#define __BSP_IMU_EKF_H

#include "attitude_common.h"
#include <stdint.h>


/*
 * ============================================================
 * EKF状态
 *
 * X =
 *
 * [q0]
 * [q1]
 * [q2]
 * [q3]
 * [bx]
 * [by]
 *
 * ============================================================
 */
typedef struct
{
    Quaternion_t q;

    float bias_gx;
    float bias_gy;

    float P[6][6];

} EKF_State_t;


/*
 * EKF参数
 */
extern float EKF_Q_quat;
extern float EKF_Q_bias;
extern float EKF_R_base;


/*
 * EKF初始化
 */
void EKF_Init(EKF_State_t *ekf);


/*
 * EKF更新
 *
 * gx gy gz：rad/s
 * ax ay az：m/s2
 * dt：s
 */
void EKF_Update(float gx,
                float gy,
                float gz,
                float ax,
                float ay,
                float az,
                float dt,
                EKF_State_t *ekf);

#endif
								