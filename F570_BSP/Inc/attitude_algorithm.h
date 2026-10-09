#ifndef __ATTITUDE_ALGORITHM_H
#define __ATTITUDE_ALGORITHM_H

#include "attitude_common.h"
#include <stdint.h>


/*
 * ============================================================
 * 姿态算法类型
 * ============================================================
 */
typedef enum
{
    ATTITUDE_ALGO_MAHONY = 0,
    ATTITUDE_ALGO_EKF    = 1

} AttitudeAlgorithmType_t;


/*
 * ============================================================
 * 姿态算法初始化
 * ============================================================
 */
void Attitude_Init(AttitudeAlgorithmType_t type);


/*
 * ============================================================
 * 切换姿态算法
 * ============================================================
 */
void Attitude_SetAlgorithm(AttitudeAlgorithmType_t type);


/*
 * ============================================================
 * 获取当前算法
 * ============================================================
 */
AttitudeAlgorithmType_t Attitude_GetAlgorithm(void);


/*
 * ============================================================
 * 统一姿态更新接口
 *
 * gx gy gz：rad/s
 * ax ay az：m/s2
 * dt：s
 * ============================================================
 */
void Attitude_Update(float gx,
                     float gy,
                     float gz,
                     float ax,
                     float ay,
                     float az,
                     float dt);


/*
 * ============================================================
 * 获取统一欧拉角
 *
 * roll  ：°
 * pitch ：°
 * yaw   ：°
 * ============================================================
 */
void Attitude_GetEuler(float *roll,
                       float *pitch,
                       float *yaw);


/*
 * ============================================================
 * 获取当前算法名称
 * ============================================================
 */
const char *Attitude_GetAlgorithmName(void);

void Attitude_GetZero(float *roll, float *pitch, float *yaw);
											 
uint8_t Attitude_IsZeroReady(void);											 
#endif
											 