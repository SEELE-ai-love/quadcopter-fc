// ============================================================
//由单任务耦合解耦成多任务+Free RTOS形式，更具工程化与时效性
// ============================================================
#include "attitude_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "Bsp_imu.h"
#include "Bsp_iic.h"
#include "icm20948_reg.h"

/* ?? 共享姿态数据（只在本文件可见） */
static float g_latest_roll  = 0.0f;
static float g_latest_pitch = 0.0f;
static float g_latest_yaw   = 0.0f;

/* ?? ?? 跨任务接口：加临界区，防止读到半新半旧的数据 */
void Attitude_GetLatest(float *roll, float *pitch, float *yaw)
{
	  /* 进入临界区，关闭中断 */
    taskENTER_CRITICAL();
	
    *roll  = g_latest_roll;
    *pitch = g_latest_pitch;
    *yaw   = g_latest_yaw;
	
	 /* 推出临界区，开启中断 */
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

    /* ?? 初始化失败就重试，避免带着错误状态继续跑 */
    while (!ICM20948_Init(&hi2c1)) {
        printf("ICM20948 init failed, retrying...\r\n");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    printf("ICM20948 init success\r\n");

    for (;;)
    {
        float dt = 1.0f / (float)ATTITUDE_TASK_FREQ;

        if (ICM20948_Update(&imu, &hi2c1))
        {
            /*  姿态解算：四元数 + Mahony */
            Quaternation_Update(imu.gyro.x, imu.gyro.y, imu.gyro.z,
                                 imu.accel.x, imu.accel.y, imu.accel.z,
                                 dt, &attitude);
            Quaternion_ToEuler(&attitude, &roll, &pitch, &yaw);

            /* 进入临界区，关闭中断 */
            taskENTER_CRITICAL();
					
            g_latest_roll  = roll;
            g_latest_pitch = pitch;
            g_latest_yaw   = yaw;
					
					  /* 推出临界区，开启中断 */
            taskEXIT_CRITICAL();
            printf("%.2f,%.2f,%.2f\r\n", roll, pitch, yaw);
        }
        else
        {
            printf("IMU read failed, skip this cycle\r\n");
        }

        /* 固定200Hz周期 */
				/*加入preTimr之后，等到上一拍结束之后，在开始5ms延时*/
        vTaskDelayUntil(&preTime, pdMS_TO_TICKS(1000.0f / ATTITUDE_TASK_FREQ));
    }
}
