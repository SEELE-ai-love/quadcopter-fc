// ============================================================
// 由单任务耦合解耦成多任务 + FreeRTOS 形式
// ============================================================

#include "attitude_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdio.h>

#include "Bsp_imu.h"
#include "Bsp_iic.h"
#include "icm20948_reg.h"
#include "attitude_algorithm.h"
#include "pid.h"
#include "math.h"
#define GYRO_CALIBRATE_NUM 2000
/* ============================================================
 * 共享姿态数据
 * ============================================================ */
static float g_latest_roll  = 0.0f;
static float g_latest_pitch = 0.0f;
static float g_latest_yaw   = 0.0f;

static float g_latest_gx = 0.0f;
static float g_latest_gy = 0.0f;
static float g_latest_gz = 0.0f;

float gyro_sum_x = 0.0f;
float gyro_sum_y = 0.0f;
float gyro_sum_z = 0.0f;

/* ============================================================
 * printf 互斥锁
 * ============================================================ */
extern SemaphoreHandle_t g_xMutex_Printf;


/* ============================================================
 * 跨任务接口：姿态角
 * ============================================================ */
void Attitude_GetLatest(float *roll,
                        float *pitch,
                        float *yaw)
{
    taskENTER_CRITICAL();

    *roll  = g_latest_roll;
    *pitch = g_latest_pitch;
    *yaw   = g_latest_yaw;

    taskEXIT_CRITICAL();
}


/* ============================================================
 * 跨任务接口：角速度
 * ============================================================ */
void Attitude_GetGyro(float *gx,
                      float *gy,
                      float *gz)
{
    taskENTER_CRITICAL();

    *gx = g_latest_gx;
    *gy = g_latest_gy;
    *gz = g_latest_gz;

    taskEXIT_CRITICAL();
}


/* ============================================================
 * 姿态任务
 *
 * 200 Hz
 * dt = 0.005s
 *
 * 当前使用 Mahony
 * ============================================================ */
void attitude_task(void *param)
{
    IMU_DATA_T imu = {0};

    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    float zero_roll  = 0.0f;
    float zero_pitch = 0.0f;
    float zero_yaw   = 0.0f;

    TickType_t preTime;

    uint32_t print_divider = 0U;

    (void)param;


    /* ========================================================
     * 姿态算法选择
     *
     * Mahony / EKF 二选一
     * ======================================================== */
    //Attitude_Init(ATTITUDE_ALGO_MAHONY);
    Attitude_Init(ATTITUDE_ALGO_EKF);


    /* ========================================================
     * 初始化 ICM20948
     * ======================================================== */
    while (!ICM20948_Init(&hi2c1))
    {
        vTaskDelay(pdMS_TO_TICKS(200));
    }


    /* ========================================================
     * 获取任务起始时间
     * ======================================================== */
    preTime = xTaskGetTickCount();


    /* ========================================================
     * 主循环
     * ======================================================== */
    for (;;)
    {
        float dt = 1.0f / (float)ATTITUDE_TASK_FREQ;


        /* ====================================================
         * 读取 IMU
         * ==================================================== */
        if (ICM20948_Update(&imu, &hi2c1))
        {
            /* =================================================
             * 姿态解算
             *
             * gyro：rad/s
             * accel：当前 Bsp_imu.c 的单位
             * ================================================= */
            Attitude_Update(imu.gyro.x,
                            imu.gyro.y,
                            imu.gyro.z,
                            imu.accel.x,
                            imu.accel.y,
                            imu.accel.z,
                            dt);


            /* =================================================
             * 获取最终姿态角
             *
             * 这里已经经过：
             *
             * 原始姿态
             *     ↓
             * AttitudeZero
             *     ↓
             * 最终姿态
             * ================================================= */
            Attitude_GetEuler(&roll,
                              &pitch,
                              &yaw);


            /* =================================================
             * 获取当前零点
             * ================================================= */
            Attitude_GetZero(&zero_roll,
                             &zero_pitch,
                             &zero_yaw);


            /* =================================================
             * 发布最终姿态给 control_task
             *
             * 注意：
             * 这里保留你原来的 Roll / Pitch 方向定义
             * ================================================= */
            taskENTER_CRITICAL();

            g_latest_roll  = roll;
            g_latest_pitch = pitch;
            g_latest_yaw   = yaw;

            /*
             * 当前 gyro 仍然保持 Bsp_imu.c 的单位：
             * rad/s
             */
            g_latest_gx = imu.gyro.x;
            g_latest_gy = imu.gyro.y;
            g_latest_gz = imu.gyro.z;

            taskEXIT_CRITICAL();


            /* =================================================
             * 串口打印
             *
             * 200Hz / 20 = 10Hz
             * 每秒打印10次
             *
             * %.1f：
             * 保留 0.1°
             * ================================================= */
            print_divider++;
     /* =================================================
             * 串口打印
             *
             * 验证最基本的单位是否正确，串级PID统一成弧度制
             * gyro的rad\s和dps是否成功统一
             * accel加速度是否是m\s²
             * ACC_NORM ≈ 9.8 m/s²，说命加速度的缩放没有问题
             * 
       * ================================================= */
//static uint8_t print_cnt = 0;

//print_cnt++;

//if(print_cnt >= 40)   // 200Hz / 40 = 5Hz
//{
//    print_cnt = 0;

//    float gyro_x_dps = imu.gyro.x * 57.2957795f;
//    float gyro_y_dps = imu.gyro.y * 57.2957795f;
//    float gyro_z_dps = imu.gyro.z * 57.2957795f;

//    float accel_norm =
//        sqrtf(imu.accel.x * imu.accel.x +
//              imu.accel.y * imu.accel.y +
//              imu.accel.z * imu.accel.z);

//    printf("BASIC | "
//           "ACC[m/s2]=%.3f %.3f %.3f | "
//           "ACC_NORM=%.3f | "
//           "GYRO[rad/s]=%.5f %.5f %.5f | "
//           "GYRO[dps]=%.3f %.3f %.3f\r\n",
//           imu.accel.x,
//           imu.accel.y,
//           imu.accel.z,
//           accel_norm,
//           imu.gyro.x,
//           imu.gyro.y,
//           imu.gyro.z,
//           gyro_x_dps,
//           gyro_y_dps,
//           gyro_z_dps);
//}
            if (print_divider >=
                (ATTITUDE_TASK_FREQ / 10U))
            {
                print_divider = 0U;


                if (xSemaphoreTake(g_xMutex_Printf,
                                   portMAX_DELAY) == pdTRUE)
                {
//                    printf("Angle: R=%+.3f P=%+.3f Y=%+.3f | "
//                           "gyro: x=%+.3f y=%+.3f z=%+.3f | "
//                           "READY=%d\r\n",
//                           roll,
//                           pitch,
//                           yaw,
//                           imu.gyro.x,
//                           imu.gyro.y,
//                           imu.gyro.z,
//                           Attitude_IsZeroReady());
									
									

                    xSemaphoreGive(g_xMutex_Printf);
                }
            }
        }


        /* ====================================================
         * 保持 200 Hz
         * ==================================================== */
        vTaskDelayUntil(&preTime,
                        pdMS_TO_TICKS(
                        1000.0f /
                        (float)ATTITUDE_TASK_FREQ));
    }
}

