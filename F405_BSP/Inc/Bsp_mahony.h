#ifndef __BSP_MAHONY_H
#define __BSP_MAHONY_H

#include "attitude_common.h"


/*
 * ============================================================
 * Mahony初始化
 * ============================================================
 */
void Mahony_Init(void);


/*
 * ============================================================
 * Mahony更新
 *
 * gx gy gz：
 * 陀螺仪角速度，单位 rad/s
 *
 * ax ay az：
 * 加速度，单位 m/s2
 *
 * dt：
 * 采样周期，单位 s
 * ============================================================
 */
void Mahony_Update(float gx,
                   float gy,
                   float gz,
                   float ax,
                   float ay,
                   float az,
                   float dt);


/*
 * ============================================================
 * 获取Mahony当前四元数
 * ============================================================
 */
Quaternion_t Mahony_GetQuaternion(void);

#endif
									 