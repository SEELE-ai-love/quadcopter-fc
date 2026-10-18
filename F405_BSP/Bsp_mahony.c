#include "Bsp_mahony.h"
#include "Bsp_imu.h"
#include <math.h>

// ==========================================
// 读取加速度原始值，直接算角度
// ==========================================
#define radian 57.2958f
#define EINT_LIMIT 0.3f
//加速度计可信区间判断
#define ACCEL_TRUST_LOW 0.85f
#define ACCEL_TRUST_HIGH 1.15f

//定义全局变量
float Kp=0.8f;
float Ki=0.0001f;


/*
 * ============================================================
 * Mahony自己的积分误差
 * ============================================================
 */
static float g_mahony_eInt[3] = {0.0f, 0.0f, 0.0f};


/*
 * ============================================================
 * Mahony自己的四元数
 * ============================================================
 */
static Quaternion_t g_mahony_q =
{
    1.0f,
    0.0f,
    0.0f,
    0.0f
};


/*
 * ============================================================
 * 初始化Mahony
 * ============================================================
 */
void Mahony_Init(void)
{
    /*
     * 四元数恢复单位四元数
     */
    g_mahony_q.q0 = 1.0f;
    g_mahony_q.q1 = 0.0f;
    g_mahony_q.q2 = 0.0f;
    g_mahony_q.q3 = 0.0f;

    /*
     * 清除积分误差
     */
    g_mahony_eInt[0] = 0.0f;
    g_mahony_eInt[1] = 0.0f;
    g_mahony_eInt[2] = 0.0f;
}


/*
 * ============================================================
 * 使用算法：牛顿迭代计算1/sqrt
 * ============================================================
 */
static float insprt(float x)
{
    float halfx=0.5f*x;
    float y=x;
    long i=*(long *)&y;

    i=0x5f3759df-(i>>1);

    y=*(float*)&i;

    y=y*(1.5f-halfx *y*y);

    return y;
}


/*
 * ============================================================
 * Mahony姿态更新
 * ============================================================
 */
void Mahony_Update(float gx,
                   float gy,
                   float gz,
                   float ax,
                   float ay,
                   float az,
                   float dt)
{
    //更新四元数
    float q0=g_mahony_q.q0;
    float q1=g_mahony_q.q1;
    float q2=g_mahony_q.q2;
    float q3=g_mahony_q.q3;

    //归一化加速度计
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);

    /*
     * 【新增】
     * ICM20948_Update()输出的加速度单位是m/s2。
     * 静止时norm_a约为9.81。
     *
     * 原来的0.85~1.15判断是针对归一化后的重力值。
     * 因此这里先转换成g值判断可信度。
     */
    float norm_g = norm_a / 9.81f;

    if(norm_g >= ACCEL_TRUST_LOW &&
       norm_g <= ACCEL_TRUST_HIGH)
    {
        if (norm_a > 1e-12f)
        {
            ax /= norm_a;
            ay /= norm_a;
            az /= norm_a;
        }
        else
        {
            ax = 0.0f;
            ay = 0.0f;
            az = 0.0f;
        }
    }
    else
    {
        /*
         * 加速度不可信时，不使用本次加速度修正。
         */
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;
    }

    //反推理论重力
    float vx=2.0f*(q1*q3-q0*q2);
    float vy=2.0f*(q0*q1+q2*q3);
    float vz=1.0f-2.0f*(q1*q1+q2*q2);

    //叉乘算误差
    float ex=ay*vz-az*vy;
    float ey=az*vx-ax*vz;
    float ez=ax*vy-ay*vx;

    //积分误差，对误差进叠加
    g_mahony_eInt[0] +=ex;
    g_mahony_eInt[1] +=ey;
    g_mahony_eInt[2] +=ez;


//float Kp = 0.8f;
//float Ki = 0.0001f;
    gx += Kp * ex + Ki * g_mahony_eInt[0];
    gy += Kp * ey + Ki * g_mahony_eInt[1];
    gz += Kp * ez + Ki * g_mahony_eInt[2];


    float halft=0.5f*dt;
    //四元数矩阵微分运算
    //
    float q0_dot = (-q1 * gx - q2 * gy - q3 * gz) * halft;
    float q1_dot = ( q0 * gx + q2 * gz - q3 * gy) * halft;
    float q2_dot = ( q0 * gy - q1 * gz + q3 * gx) * halft;
    float q3_dot = ( q0 * gz + q1 * gy - q2 * gx) * halft;

    //更新四元数

    q0+=q0_dot;
    q1+=q1_dot;
    q2+=q2_dot;
    q3+=q3_dot;

    //四元数归一化
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm > 1e-12f)
    {
        g_mahony_q.q0 = q0 / norm;
        g_mahony_q.q1 = q1 / norm;
        g_mahony_q.q2 = q2 / norm;
        g_mahony_q.q3 = q3 / norm;
    }
    else
    {
        g_mahony_q.q0 = 1.0f;
        g_mahony_q.q1 = 0.0f;
        g_mahony_q.q2 = 0.0f;
        g_mahony_q.q3 = 0.0f;
    }
}


/*
 * ============================================================
 * 获取Mahony四元数
 * ============================================================
 */
Quaternion_t Mahony_GetQuaternion(void)
{
    return g_mahony_q;
}
