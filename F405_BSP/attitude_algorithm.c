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
/* 原始姿态角：算法直接算出来的角度 */
static float g_raw_roll  = 0.0f;
static float g_raw_pitch = 0.0f;
static float g_raw_yaw   = 0.0f;

/* 最终输出姿态角：原始角度 - 零点 */
static float g_roll  = 0.0f;
static float g_pitch = 0.0f;
static float g_yaw   = 0.0f;
/*
 * ============================================================
 * 姿态启动零点
 * ============================================================
 */
static ATTITUDE_DATA_T AttitudeZero = {0};



/*
 * ============================================================
 * 姿态零点采样
 * ============================================================
 */
/* 开机先等100帧(0.5s)让EKF/Mahony收敛，再采200帧(1s)取平均做零点 */
#define ATTITUDE_ZERO_WAIT_NUM   100
#define ATTITUDE_ZERO_SAMPLE_NUM 200

static uint16_t attitude_zero_wait_count = 0;
static uint16_t attitude_zero_sample_count = 0;

static float attitude_roll_sum  = 0.0f;
static float attitude_pitch_sum = 0.0f;
static float attitude_yaw_sum   = 0.0f;

static uint8_t attitude_zero_ready = 0;
static void Attitude_UpdateZero(void);
static void Attitude_ApplyZero(void);
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


    /* Roll */
    g_raw_roll =
        atan2f(2.0f * (q0 * q1 + q2 * q3),
               1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* Pitch */
    float pitch_value =
        2.0f * (q0 * q2 - q3 * q1);

    if (pitch_value > 1.0f)
        pitch_value = 1.0f;

    if (pitch_value < -1.0f)
        pitch_value = -1.0f;

    g_raw_pitch = asinf(pitch_value);

    /* Yaw */
    g_raw_yaw =
        atan2f(2.0f * (q0 * q3 + q1 * q2),
               1.0f - 2.0f * (q2 * q2 + q3 * q3));

    /* 默认先让最终输出等于原始值 */
    g_roll  = g_raw_roll;
    g_pitch = g_raw_pitch;
    g_yaw   = g_raw_yaw;
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

    attitude_zero_wait_count = 0;
    attitude_zero_sample_count = 0;

    attitude_roll_sum = 0.0f;
    attitude_pitch_sum = 0.0f;
    attitude_yaw_sum = 0.0f;

    attitude_zero_ready = 0;

    AttitudeZero.roll = 0.0f;
    AttitudeZero.pitch = 0.0f;
    AttitudeZero.yaw = 0.0f;

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
        Mahony_Update(gx, gy, gz,
                      ax, ay, az,
                      dt);

        Quaternion_t temp_q = Mahony_GetQuaternion();

        Attitude_QuaternionToEuler(&temp_q);
    }
    else
    {
        EKF_Update(gx, gy, gz,
                   ax, ay, az,
                   dt,
                   &g_ekf);

        Attitude_QuaternionToEuler(&g_ekf.q);
    }

    /* 原始姿态角已经算完 */

    if (!attitude_zero_ready)
    {
        Attitude_UpdateZero();
    }

    /* 零点采样完成后，对当前姿态进行补偿 */
    Attitude_ApplyZero();

}


static void Attitude_UpdateZero(void)
{
    /* 前500ms只等待姿态算法稳定 */
    if (attitude_zero_wait_count < ATTITUDE_ZERO_WAIT_NUM)
    {
        attitude_zero_wait_count++;
        return;
    }

    /* 开始正式采样 */
    if (attitude_zero_sample_count < ATTITUDE_ZERO_SAMPLE_NUM)
    {
        attitude_roll_sum  += g_raw_roll;
        attitude_pitch_sum += g_raw_pitch;
        attitude_yaw_sum   += g_raw_yaw;

        attitude_zero_sample_count++;

        /* 200个样本全部采完 */
        if (attitude_zero_sample_count >= ATTITUDE_ZERO_SAMPLE_NUM)
        {
            AttitudeZero.roll =
                attitude_roll_sum / (float)ATTITUDE_ZERO_SAMPLE_NUM;

            AttitudeZero.pitch =
                attitude_pitch_sum / (float)ATTITUDE_ZERO_SAMPLE_NUM;

            AttitudeZero.yaw =
                attitude_yaw_sum / (float)ATTITUDE_ZERO_SAMPLE_NUM;

            attitude_zero_ready = 1;
        }

        return;
    }
}

static void Attitude_ApplyZero(void)
{
    if (!attitude_zero_ready)
    {
        g_roll  = g_raw_roll;
        g_pitch = g_raw_pitch;
        g_yaw   = g_raw_yaw;
        return;
    }

    g_roll  = g_raw_roll  - AttitudeZero.roll;
    g_pitch = g_raw_pitch - AttitudeZero.pitch;
    g_yaw   = g_raw_yaw   - AttitudeZero.yaw;
}

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
                       float *yaw)
{


    if (roll != NULL)
    {
        *roll = -g_roll;
    }



    if (pitch != NULL)
    {
        *pitch = -g_pitch;
    }



    if (yaw != NULL)
    {
        *yaw = g_yaw;
    }

}

/*
 * ============================================================
 * 零点获取函数
 *
 * ============================================================
 */
void Attitude_GetZero(float *roll,
                      float *pitch,
                      float *yaw)
{
    if (roll != NULL)
        *roll = AttitudeZero.roll;

    if (pitch != NULL)
        *pitch = AttitudeZero.pitch;

    if (yaw != NULL)
        *yaw = AttitudeZero.yaw;
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

uint8_t Attitude_IsZeroReady(void)
{
    return attitude_zero_ready;
}
