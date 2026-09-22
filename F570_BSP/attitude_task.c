// ============================================================
//由单任务耦合解耦成多任务+Free RTOS形式，更具工程化与时效性
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

/* ?? 共享姿态数据（只在本文件可见） */
static float g_latest_roll  = 0.0f;
static float g_latest_pitch = 0.0f;
static float g_latest_yaw   = 0.0f;

//========== 互斥锁 ==========//
extern SemaphoreHandle_t g_xMutex_Printf;


/*跨任务接口：加临界区，防止读到半新半旧的数据 */
void Attitude_GetLatest(float *roll, float *pitch, float *yaw)
{
    /* 进入临界区，关闭中断 */
    taskENTER_CRITICAL();

    *roll  = g_latest_roll;
    *pitch = g_latest_pitch;
    *yaw   = g_latest_yaw;

    /* 退出临界区，开启中断 */
    taskEXIT_CRITICAL();
}


/* ?? 姿态任务 */
void attitude_task(void *param)
{
    IMU_DATA_T imu = {0};
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    TickType_t preTime;//pretime作为RTOS里面专门记录时间戳的变量

    (void)param;

    preTime = xTaskGetTickCount();//获取当前系统Tick数


    /*
     * ========================================================
     * 姿态算法选择
     *
     * 当前使用：
     *
     * ATTITUDE_ALGO_MAHONY
     *
     * 如果需要测试EKF，改成：
     *
     * ATTITUDE_ALGO_EKF
     * ========================================================
     */
    //Attitude_Init(ATTITUDE_ALGO_MAHONY);//这个是切换算法的接口，如果要切换到EKF卡尔曼滤波就把下面的注释去掉
		Attitude_Init(ATTITUDE_ALGO_EKF);


    /* ?? 初始化失败就重试，避免带着错误状态继续跑 */
    while (!ICM20948_Init(&hi2c1))
    {
        if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
        {
            printf("ICM20948 init failed, retrying...\r\n");
            xSemaphoreGive(g_xMutex_Printf);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }


    // ========== 互斥锁保护 printf ==========
    if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
    {
        printf("ICM20948 init success\r\n");
        xSemaphoreGive(g_xMutex_Printf);
    }


    for (;;)
    {
        float dt = 1.0f / (float)ATTITUDE_TASK_FREQ;

        if (ICM20948_Update(&imu, &hi2c1))
        {
            /*
             * ==================================================
             * 姿态解算
             *
             * 这里不再直接调用Mahony或者EKF。
             *
             * 由统一接口根据当前选择的算法进行计算。
             * ==================================================
             */
            Attitude_Update(imu.gyro.x,
                            imu.gyro.y,
                            imu.gyro.z,
                            imu.accel.x,
                            imu.accel.y,
                            imu.accel.z,
                            dt);


            /*
             * ==================================================
             * 获取统一姿态角
             * ==================================================
             */
            Attitude_GetEuler(&roll,
                              &pitch,
                              &yaw);


            /* 进入临界区，关闭中断 */
            taskENTER_CRITICAL();

            g_latest_roll  = roll;
            g_latest_pitch = pitch;
            g_latest_yaw   = yaw;

            /* 推出临界区，开启中断 */
            taskEXIT_CRITICAL();


            if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
            {
                /*
                 * 小数点后三位
                 */
                printf("%.3f,%.3f,%.3f\r\n",
                       roll,
                       pitch,
                       yaw);

                xSemaphoreGive(g_xMutex_Printf);
            }
        }
        else
        {
            //printf("IMU read failed, skip this cycle\r\n");
        }

        /* 固定200Hz周期 */
        /*加入preTimr之后，等到上一拍结束之后，在开始5ms延时*/
        vTaskDelayUntil(&preTime,
                        pdMS_TO_TICKS(1000.0f / ATTITUDE_TASK_FREQ));
    }
}
