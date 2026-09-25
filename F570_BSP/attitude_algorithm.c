#include "attitude_algorithm.h"

#include "Bsp_mahony.h"
#include "Bsp_imu_ekf.h"
#include "Bsp_imu.h"

#include <stdio.h>
#include <math.h>


/*
 * ============================================================
 * 当前使用的姿态算法
 *
 * 默认使用Mahony
 * ============================================================
 */
static AttitudeAlgorithmType_t g_attitude_algorithm =
    ATTITUDE_ALGO_MAHONY;


/*
 * ============================================================
 * EKF独立状态
 * ============================================================
 */
static EKF_State_t g_ekf;


/*
 * ============================================================
 * 统一姿态输出
 * ============================================================
 */
static float g_roll  = 0.0f;
static float g_pitch = 0.0f;
static float g_yaw   = 0.0f;


/*
 * ============================================================
 * 四元数转欧拉角
 * ============================================================
 */
static void Attitude_QuaternionToEuler(const Quaternion_t *q)
{
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;

    const float RAD_TO_DEG = 57.2958f;


    /*
     * Roll是滚转角
     */
    g_roll =
        atan2f(2.0f * (q0 * q1 + q2 * q3),
               1.0f - 2.0f * (q1 * q1 + q2 * q2))
        * RAD_TO_DEG;


    /*
     * Pitch是俯仰角
     */
    float pitch_value =
        2.0f * (q0 * q2 - q3 * q1);

    /*
     * 防止浮点误差导致asinf输入超过[-1,1]
     */
    if (pitch_value > 1.0f)
    {
        pitch_value = 1.0f;
    }
    else if (pitch_value < -1.0f)
    {
        pitch_value = -1.0f;
    }

    g_pitch = asinf(pitch_value) * RAD_TO_DEG;


    /*
     * Yaw是偏航角（这个需要磁力计介入）
     */
    g_yaw =
        atan2f(2.0f * (q0 * q3 + q1 * q2),
               1.0f - 2.0f * (q2 * q2 + q3 * q3))
        * RAD_TO_DEG;


    /*
     * 保留你原来的机架安装角度修正
     */
    g_roll  -= ZeroAttitude.roll;
    g_pitch -= ZeroAttitude.pitch;
    g_yaw   -= ZeroAttitude.yaw;
}


/*
 * ============================================================
 * 初始化
 * ============================================================
 */
void Attitude_Init(AttitudeAlgorithmType_t type)
{
    g_attitude_algorithm = type;

    g_roll  = 0.0f;
    g_pitch = 0.0f;
    g_yaw   = 0.0f;


    if (type == ATTITUDE_ALGO_MAHONY)
    {
        Mahony_Init();
    }
    else
    {
        EKF_Init(&g_ekf);
    }
}


/*
 * ============================================================
 * 切换姿态算法
 * ============================================================
 */
void Attitude_SetAlgorithm(AttitudeAlgorithmType_t type)
{
    /*
     * 切换算法时重新初始化对应算法状态
     */
    Attitude_Init(type);
}


/*
 * ============================================================
 * 获取当前算法
 * ============================================================
 */
AttitudeAlgorithmType_t Attitude_GetAlgorithm(void)
{
    return g_attitude_algorithm;
}


/*
 * ============================================================
 * 统一姿态更新接口
 * ============================================================
 */
void Attitude_Update(float gx,
                     float gy,
                     float gz,
                     float ax,
                     float ay,
                     float az,
                     float dt)
{
    /*
     * ========================================================
     * Mahony
     * ========================================================
     */
	
    if (g_attitude_algorithm == ATTITUDE_ALGO_MAHONY)
    {
        Mahony_Update(gx,
                      gy,
                      gz,
                      ax,
                      ay,
                      az,
                      dt);

        Quaternion_t temp_q = Mahony_GetQuaternion();
        Attitude_QuaternionToEuler(&temp_q);
    }


    /*
     * ========================================================
     * EKF
     * ========================================================
     */
    else
    {
        EKF_Update(gx,
                   gy,
                   gz,
                   ax,
                   ay,
                   az,
                   dt,
                   &g_ekf);
//测试飞控板子平放时候的零偏
//			printf("EKF bx=%.6f, by=%.6f\r\n",
//       g_ekf.bias_gx,
//       g_ekf.bias_gy);

        Attitude_QuaternionToEuler(
            &g_ekf.q);
    }
}


/*
 * ============================================================
 * 获取统一欧拉角
 * ============================================================
 */
void Attitude_GetEuler(float *roll,
                       float *pitch,
                       float *yaw)
{
    if (roll != NULL)
    {
        *roll =-g_roll;
    }

    if (pitch != NULL)
    {
        *pitch =-g_pitch;
    }

    if (yaw != NULL)
    {
        *yaw = g_yaw;
    }
}


/*
 * ============================================================
 * 获取当前算法名称
 * ============================================================
 */
const char *Attitude_GetAlgorithmName(void)
{
    if (g_attitude_algorithm == ATTITUDE_ALGO_MAHONY)
    {
        return "Mahony";
    }

    return "EKF";
}
