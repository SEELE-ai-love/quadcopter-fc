/* ============================================================================
 * Bsp_imu_ekf.c —— 扩展卡尔曼滤波(EKF)姿态解算，注释都是自己写的
 * ----------------------------------------------------------------------------
 * 跟 Bsp_imu.c 里的 Mahony 版本并列存在，互不影响：
 *   - Mahony：Quaternation_Update() + Quaternion_ToEuler()
 *   - EKF：  EKF_Init() + EKF_Update() + 复用同一个 Quaternion_ToEuler()
 * ============================================================================ */

#include "Bsp_imu_ekf.h"
#include <math.h>          // 与Mahony的姿态解算不同，这里的math包含sqrtf和fabsf
#include <string.h>        // 用于调用memset和memcpy
#include <stdio.h>


// 这个是可调的变量
// 对于多变量扩展卡尔曼而言：
// Q表示算法预测过程中的不确定性
// R表示真实传感器测量的不确定性
float EKF_Q_quat = 1e-6f;  // 表示四元数部分的预测误差
float EKF_Q_bias = 1e-9f;  // 表示陀螺仪零偏的预测误差
float EKF_R_base = 0.01f;  // 表示加速度传感器测量的误差


/* 加速度计可信区间——跟Bsp_imu.c里Mahony的判断逻辑一致，
 * 但这里不是"区间外直接跳过"，而是"偏离越多，测量噪声R越大，
 * 卡尔曼增益K自动跟着变小"——这是EKF相对Mahony的核心区别
 */
#define EKF_ACCEL_TRUST_LOW  0.85f
#define EKF_ACCEL_TRUST_HIGH 1.15f


/* ============================================================================
 * 小矩阵运算——尺寸全部写死，不用通用矩阵库，
 * 跟项目里 insprt()、dshot_crc() 一样的原则：手写、直接、不引入额外依赖
 * ============================================================================ */


/*
 * 两个6x6矩阵运算
 *
 * 这个函数在后面会被调两次：
 *
 * 第一次：
 *     F * P
 *
 * 第二次：
 *     (F * P) * F^T
 *
 * 两次连起来就是：
 *
 *     F * P * F^T
 */
static void Mat6x6_Mult(
    const float A[6][6],
    const float B[6][6],
    float C[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}


/*
 * 两个6x6矩阵行列转化
 * 专门用于计算：
 *     F^T
 */
static void Mat6x6_Transpose(
    const float A[6][6],
    float At[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            /*
             * 转置：
             *
             * 原来的 A[i][j]
             * ↓
             * 转置后的 At[j][i]
             */
            At[j][i] = A[i][j];
        }
    }
}

/*
 * 把3x6的H转化成6x3的Ht
 */
static void Mat3x6_Transpose(
    const float H[3][6],
    float Ht[6][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            Ht[j][i] = H[i][j];
        }
    }
}


/*
 * 一个3x6矩阵乘6x6矩阵
 * 专门用于：
 *     H * P_pred
 */
static void Mat3x6_Mult_6x6(
    const float H[3][6],
    const float P[6][6],
    float HP[3][6])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float sum = 0.0f;

            for (int k = 0; k < 6; k++)
            {
                sum += H[i][k] * P[k][j];
            }

            HP[i][j] = sum;
        }
    }
}

/*
 * 一个3x6乘6x3矩阵
 * 专门用于：
 *     H * P * H^T
 * 加上R之后就是：
 *     S = H * P * H^T + R
 * 也是卡尔曼增益K公式中的分母部分
 */
static void Mat3x6_Mult_6x3(
    const float HP[3][6],
    const float Ht[6][3],
    float S[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;

            for (int k = 0; k < 6; k++)
            {
                sum += HP[i][k] * Ht[k][j];
            }

            S[i][j] = sum;
        }
    }
}

/*
 * 一个6x6乘6x3矩阵
 * 专门用于：
 *     P_pred * H^T
 * 这是卡尔曼增益K公式中的分子部分
 */
static void Mat6x6_Mult_6x3(
    const float P[6][6],
    const float Ht[6][3],
    float PHt[6][3])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;

            for (int k = 0; k < 6; k++)
            {
                sum += P[i][k] * Ht[k][j];
            }

            PHt[i][j] = sum;
        }
    }
}

/*
 * 一个6x3乘3x3的矩阵
 * 专门用于：
 *     P * H^T * S^-1
 * 最终拼出完整的卡尔曼增益K
 */
static void Mat6x3_Mult_3x3(
    const float PHt[6][3],
    const float Sinv[3][3],
    float K[6][3])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                sum += PHt[i][k] * Sinv[k][j];
            }
            K[i][j] = sum;
        }
    }
}


/*
 * 一个6x3乘3x1矩阵
 * 专门用于：
 *     K * y
 * 也就是最终要加到状态量上的修正量：
 *     dx
 */
static void Mat6x3_Mult_3x1(
    const float K[6][3],
    const float y[3],
    float dx[6])
{
    for (int i = 0; i < 6; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < 3; k++)
        {
            sum += K[i][k] * y[k];
        }
        dx[i] = sum;
    }
}

/*
 * 一个6x3乘3x6矩阵
 * 专门用于：
 *     K * H * P
 * 当前调用时：
 *     HP = H * P_pred
 * 所以：
 *     K * HP
 * 就是：
 *     K * H * P_pred
 * 最终：
 *     P = P - KHP
 */
static void Mat6x3_Mult_3x6(
    const float K[6][3],
    const float HP[3][6],
    float KHP[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float sum = 0.0f;

            for (int k = 0; k < 3; k++)
            {
                sum += K[i][k] * HP[k][j];
            }

            KHP[i][j] = sum;
        }
    }
}


/*
 * 3x3矩阵求逆——伴随矩阵法
 *
 * 行列式接近0说明矩阵奇异。
 *
 * 出现大概率是Q/R参数设得太极端，
 * 直接跳过本次修正，保留预测值，
 * 比硬算出一个错误的逆矩阵导致姿态突然发散要安全。
 */
static uint8_t Mat3x3_Invert(
    const float S[3][3],
    float Sinv[3][3])
{
    /*
     * 计算3x3矩阵S的行列式
     */
    float det =
        S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
        S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
        S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);


    /*
     * 如果行列式接近0，
     * 说明S不可逆。
     */
    if (fabsf(det) < 1e-12f)
    {
        return 0;
    }


    /* 行列式的倒数 */
    float invdet = 1.0f / det;


    /*
     * 计算S的逆矩阵
     */
    Sinv[0][0] =
        (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invdet;

    Sinv[0][1] =
        -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invdet;

    Sinv[0][2] =
        (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invdet;


    Sinv[1][0] =
        -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invdet;

    Sinv[1][1] =
        (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invdet;

    Sinv[1][2] =
        -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invdet;


    Sinv[2][0] =
        (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invdet;

    Sinv[2][1] =
        -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invdet;

    Sinv[2][2] =
        (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invdet;


    return 1;
}


/*
 * 四元数初始化
 *
 * EKF_State_t结构体：
 *
 * typedef struct
 * {
 *     Quaternion_t q;
 *     float bias_gx;
 *     float bias_gy;
 *     float P[6][6];
 * } EKF_State_t;
 */
void EKF_Init(EKF_State_t *ekf)
{
    /*
     * 四元数设成单位四元数：
     * q = [1, 0, 0, 0]
     * 表示一开始假设姿态为初始参考姿态。
     */
    ekf->q.q0 = 1.0f;//四元数赋值一个1是常识，因为是四维空间的一个单位向量来表示三维运动
    ekf->q.q1 = 0.0f;
    ekf->q.q2 = 0.0f;
    ekf->q.q3 = 0.0f;
    /*
     * 两个零偏初始值都设成0。
     * 表示一开始假设陀螺仪没有零偏。
     */
    ekf->bias_gx = 0.0f;
    ekf->bias_gy = 0.0f;


    memset(ekf->P, 0, sizeof(ekf->P));//P（预测的不确定度）矩阵清零


    /*
     * 给四元数部分的协方差设置初始值。
     * P[0][0]
     * P[1][1]
     * P[2][2]
     * P[3][3]
     */
    for (int i = 0; i < 4; i++)
    {
        ekf->P[i][i] = 1e-3f;//循环四次是四元数部分
    }
    ekf->P[4][4] = 1e-4f;//后面两个是bx和by
    ekf->P[5][5] = 1e-4f;
    
 //此时P的非对角线还是0， 一开始还没有q与bias之间的协方差耦合，后面通过F：P_pred = F * P * F^T + Q 才会逐渐产生状态之间的相关性。
}

//重要部分：卡尔曼开始介入四元数
void EKF_Update(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt,
    EKF_State_t *ekf)
{
    // 读取当前四元数。
    float q0 = ekf->q.q0;
    float q1 = ekf->q.q1;
    float q2 = ekf->q.q2;
    float q3 = ekf->q.q3;
    
    // 读取当前陀螺仪零偏
    float bx = ekf->bias_gx;
    float by = ekf->bias_gy;
    
       //零偏修正后的角速度
       //Z轴目前没有加入零偏估计，
       //所以直接使用原始gz。
    float wx = gx - bx;
    float wy = gy - by;
    float wz = gz;


    /* ---------- 第一部分：先验预测 x_k- ---------- */


    /*
     * 四元数微分方程：
     * q_dot = 0.5 * Ω(w) * q
     * 这里与Mahony中的四元数运动方程保持一致。
     */
		 float q0_dot = 0.5f * (-q1*wx - q2*wy - q3*wz);
     float q1_dot = 0.5f * ( q0*wx - q3*wy + q2*wz);
     float q2_dot = 0.5f * ( q3*wx + q0*wy - q1*wz);
     float q3_dot = 0.5f * (-q2*wx + q1*wy + q0*wz);

    /*
     * 一阶欧拉积分预测（先验预测）：
     * q_new = q_old + q_dot * dt
     */
    float q0_p = q0 + q0_dot * dt;
    float q1_p = q1 + q1_dot * dt;
    float q2_p = q2 + q2_dot * dt;
    float q3_p = q3 + q3_dot * dt;
    /*
     * 四元数归一化。
     * 保证：
     * q02 + q12 + q22 + q32 = 1
     */
    float norm_q = sqrtf(q0_p*q0_p + q1_p*q1_p + q2_p*q2_p + q3_p*q3_p);
    if (norm_q > 1e-12f)
    {
        q0_p /= norm_q;
        q1_p /= norm_q;
        q2_p /= norm_q;
        q3_p /= norm_q;
    }
    else
    {
         //极端异常情况下恢复单位四元数。
        q0_p = 1.0f;
        q1_p = 0.0f;
        q2_p = 0.0f;
        q3_p = 0.0f;
    }
    /*
     * 零偏建模成随机游走。
     * 当前预测阶段假设：
     * bx(k+1) = bx(k)
     * by(k+1) = by(k)
     */
    float bx_p = bx;
    float by_p = by;


    /* ---------- 第二部分：状态转移雅可比F ---------- */


    /*
     * 状态：
     * X =[q0][q1][q2][q3][bx][by],四个四元数和两个零偏
     * F：对
     *
     * 这里使用：
     *
     *     F = I + A*dt
     */
    float F[6][6];//状态转移矩阵/雅各比矩阵
    memset(F, 0, sizeof(F));//矩阵清零

//F这个地方建议手推，草稿纸手写
    /*
     * 给主对角线赋1。
     * 先得到单位矩阵I。
     */
    for (int i = 0; i < 6; i++)
    {
        F[i][i] = 1.0f;
    }

    /*
     * F的左上4x4：
     * 表示： * | 1    -0.5*wx*dt   -0.5*wy*dt   -0.5*wz*dt |
              * | 0.5*wx*dt    1     0.5*wz*dt   -0.5*wy*dt |
              * | 0.5*wy*dt  -0.5*wz*dt    1      0.5*wx*dt |
              * | 0.5*wz*dt   0.5*wy*dt  -0.5*wx*dt    1    |
     * 也就是四元数对四元数的偏导。
     * 将陀螺仪与四元数融合之后，
     * 对四元数运动方程求偏导。
     */
    F[0][1] += -0.5f * wx * dt;
    F[0][2] += -0.5f * wy * dt;
    F[0][3] += -0.5f * wz * dt;
		
    F[1][0] +=  0.5f * wx * dt;
    F[1][2] +=  0.5f * wz * dt;
    F[1][3] += -0.5f * wy * dt;
		
    F[2][0] +=  0.5f * wy * dt;
    F[2][1] += -0.5f * wz * dt;
    F[2][3] +=  0.5f * wx * dt;
		
    F[3][0] +=  0.5f * wz * dt;
    F[3][1] +=  0.5f * wy * dt;
    F[3][2] += -0.5f * wx * dt;
    /*
     * F的右上部分：
*  | 1   -0.5*wx*dt   -0.5*wy*dt   -0.5*wz*dt    0.5*q1*dt    0.5*q2*dt|
 * | 0.5*wx*dt   1    0.5*wz*dt   -0.5*wy*dt   -0.5*q0*dt    0.5*q3*dt |
 * | 0.5*wy*dt  -0.5*wz*dt   1    0.5*wx*dt    -0.5*q3*dt   -0.5*q0*dt |
 * | 0.5*wz*dt   0.5*wy*dt  -0.5*wx*dt   1     0.5*q2*dt   -0.5*q1*dt  |
 * | 0          0          0          0             1             0    |
 * | 0          0          0          0             0             1    |
     * q → bx / by
     * 因为：
     * wx = gx - bx
     * wy = gy - by
//重要部分：由于零偏bx，by的出现会干扰四元数的预测，那么状态X的每个部分都在H之后，每个部分都会被相互修正
     * 所以四元数的预测结果会受到bx、by影响。
     */
    F[0][4] +=  0.5f * q1 * dt;
    F[0][5] +=  0.5f * q2 * dt;
		
    F[1][4] += -0.5f * q0 * dt;
    F[1][5] +=  0.5f * q3 * dt;

    F[2][4] += -0.5f * q3 * dt;
    F[2][5] += -0.5f * q0 * dt;

    F[3][4] +=  0.5f * q2 * dt;
    F[3][5] += -0.5f * q1 * dt;

    F[4][4] = 1.0f;
    F[5][5] = 1.0f;


    /* ---------- 第三部分：先验协方差 ---------- */


    /*
     * 公式：
     * P_k- = F * P * F^T + Q
     */


    float Ft[6][6];
    float FP[6][6];
    float FPFt[6][6];


    /*
     * 计算F^T。
     */
    Mat6x6_Transpose(F, Ft);//Ft
    /*
     * 计算：
     * FP = F * P
     */
    Mat6x6_Mult(F, ekf->P, FP);//Fp
    /*
     * 计算：
     * F * P * F^T
     */
    Mat6x6_Mult(FP, Ft, FPFt);//FpFt

    /*
     * 计算：P_pred：
     */
    float P_pred[6][6];//P是一个6x6的矩阵
    memcpy(P_pred, FPFt, sizeof(P_pred));//将FPFt的值给P_pred
    /*
     * 加入四元数部分的过程噪声Q。
     * 关于Q：
 *     | Qq  0   0   0   0   0  |
 *     | 0   Qq  0   0   0   0 |
 * Q = | 0   0   Qq  0   0   0 |
 *     | 0   0   0   Qq  0   0 |
 *     | 0   0   0   0   Qb  0 |
 *     | 0   0   0   0   0   Qb|
     * 当前Q只加在对角线上。
     */
    for (int i = 0; i < 4; i++)
    {
        P_pred[i][i] += EKF_Q_quat;//循环四次是让四元素加Q
    }
    /*
     * 加入bx和by的过程噪声。
     */
    P_pred[4][4] += EKF_Q_bias;
    P_pred[5][5] += EKF_Q_bias;


    /* ---------- 第四部分：加速度计观测 ---------- */


    /*
     * 计算加速度大小：
     * |a| = sqrt(ax2 + ay2 + az2)
     */
float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_a > 1e-6f)
    {
        float zx = ax / norm_a;
        float zy = ay / norm_a;
        float zz = az / norm_a;
        /*
         * 根据预测四元数计算理论重力方向。
         * h(X^-)
         * 也就是测量模型。
         */
         float hx = 2.0f * (q1_p*q3_p - q0_p*q2_p);
         float hy = 2.0f * (q0_p*q1_p + q2_p*q3_p);
         float hz = 1.0f - 2.0f * (q1_p*q1_p + q2_p*q2_p);
        /*
         * 测量残差：
         * y = z - h(X^-)
         */
        float y[3] =
        {
            zx - hx,
            zy - hy,
            zz - hz
        };
        /* ---------- 第五部分：H矩阵 ---------- */
        /*
         * H(偏导数）：
 * | -2*q2    2*q3   -2*q0    2*q1    0    0 |
 * |  2*q1    2*q0    2*q3    2*q2    0    0 |
 * |   0     -4*q1   -4*q2     0      0    0 |
         */
        float H[3][6];//H是三行六列
        memset(H, 0, sizeof(H));//清零

        H[0][0] = -2.0f * q2_p;
        H[0][1] =  2.0f * q3_p;
        H[0][2] = -2.0f * q0_p;
        H[0][3] =  2.0f * q1_p;

        H[1][0] =  2.0f * q1_p;
        H[1][1] =  2.0f * q0_p;
        H[1][2] =  2.0f * q3_p;
        H[1][3] =  2.0f * q2_p;

        H[2][0] =  0.0f;
        H[2][1] = -4.0f * q1_p;
        H[2][2] = -4.0f * q2_p;
        H[2][3] =  0.0f;
//H是求偏导数，可以自己手推
        /*
         * H的最后两列：
         * 对bx、by的偏导为0。
         * 因为当前测量模型h(q)中没有直接出现bx、by。
         * 但是bx/by仍然可以通过P中的状态相关性，
         * 最终通过K得到修正。
         */


        /* ---------- 第六部分：自适应卡尔曼滤波 ---------- */
        /*
         * 计算当前加速度大小偏离1g的程度。
         */
        float norm_g = norm_a / 9.81f;
        float accel_dev =
            fabsf(norm_g - 1.0f);
        /*
         * R缩放系数初始为1。
         */
        float r_scale = 1.0f;
        /*
         * 如果加速度大小超出可信区间，
         * 就增大R。
         * R变大：
         *     → 不信任加速度计
         *     → K变小
         *     → 减少加速度对姿态的修正
         */
        if (norm_g < EKF_ACCEL_TRUST_LOW ||
            norm_g > EKF_ACCEL_TRUST_HIGH)
        {
            r_scale =
                1.0f
                + 50.0f * accel_dev;
        }
        float r =
            EKF_R_base * r_scale;


        /* ---------- 第七部分：卡尔曼增益 ---------- */
        /*
         * 卡尔曼增益公式：
         * K=P * H^T / (H * P * H^T + R)
         */
        float Ht[6][3];
        Mat3x6_Transpose(H, Ht);
        float HP[3][6];
        Mat3x6_Mult_6x6(H, P_pred, HP);
         //计算：* S = H * P_pred * H^T
        float S[3][3];
        Mat3x6_Mult_6x3(HP, Ht, S);
        /*
         * 加入测量噪声R：
         * S = HPH^T + R
         * 当前R假设三个轴的测量噪声相同，
         * 所以只需要给S的三个对角线加r。
         */
        S[0][0] += r;
        S[1][1] += r;
        S[2][2] += r;
        /*
         * 计算S的逆矩阵，就是倒数形式
         */
        float Sinv[3][3];
        if (Mat3x3_Invert(S, Sinv))
        {
            float PHt[6][3];
            Mat6x6_Mult_6x3(P_pred, Ht, PHt);
            float K[6][3];
            Mat6x3_Mult_3x3(PHt, Sinv, K);
            /* ---------- 第八部分：后验状态估计 ---------- */

            /*
             * 状态更新公式：
             * x = x- + K*y
             */
            float dx[6];//新状态X（预测值）
            Mat6x3_Mult_3x1(
                K,
                y,
                dx
            );
            /*
             * 将修正量加入预测状态：
             * dx[0] → q0修正量
             * dx[1] → q1修正量
             * dx[2] → q2修正量
             * dx[3] → q3修正量
             * dx[4] → bx修正量
             * dx[5] → by修正量
             */
            q0_p += dx[0];
            q1_p += dx[1];
            q2_p += dx[2];
            q3_p += dx[3];
						
            bx_p += dx[4];
            by_p += dx[5];
            /*
             * 四元数归一化。因为EKF又涉及到四元数，所以再次归一化。
             */
           norm_q = sqrtf(q0_p*q0_p + q1_p*q1_p + q2_p*q2_p + q3_p*q3_p);
            if (norm_q > 1e-12f)
            {
                q0_p /= norm_q;
                q1_p /= norm_q;
                q2_p /= norm_q;
                q3_p /= norm_q;
            
						}
						
						
            /* ---------- 第九部分：协方差更新 ---------- */
            /*
             * 公式：
             * P =(I - K*H) * P_pred展开式就是：
             * P_pred - K*H*P_pred
             */
            float KHP[6][6];
            Mat6x3_Mult_3x6(K,HP,KHP);//计算KHP
						
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    P_pred[i][j] = P_pred[i][j] - KHP[i][j];//等价于P =(1 - K*H) * P_pred
                }
            }
        }
    }
		
		
    /* ---------- 第十部分：赋值成最终状态 ---------- */
    /*
     * 把最终四元数写回EKF结构体。
     */
    ekf->q.q0 = q0_p;
    ekf->q.q1 = q1_p;
    ekf->q.q2 = q2_p;
    ekf->q.q3 = q3_p;
    /*
     * 把最终陀螺仪零偏写回EKF结构体。
     */
    ekf->bias_gx = bx_p;
    ekf->bias_gy = by_p;
    /*
     * 把预测/更新后的协方差矩阵保存回正式P。
     */
    memcpy(ekf->P,P_pred,sizeof(ekf->P));//把更新后的p_pred的值赋值给P
}
