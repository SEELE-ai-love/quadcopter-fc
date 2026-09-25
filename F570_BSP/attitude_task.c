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
#include "pid.h"

/* 共享姿态数据（只在本文件可见） */
static float g_latest_roll  = 0.0f;
static float g_latest_pitch = 0.0f;
static float g_latest_yaw   = 0.0f;

static float g_latest_gx = 0.0f;
static float g_latest_gy = 0.0f;
static float g_latest_gz = 0.0f;

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

void Attitude_GetGyro(float *gx, float *gy,float *gz)
{
	  /* 进入临界区，关闭中断 */
    taskENTER_CRITICAL();

    *gx = g_latest_gx;
    *gy = g_latest_gy;
    *gz = g_latest_gz;
	
    /* 进入临界区，关闭中断 */
    taskEXIT_CRITICAL();
}

/*  姿态任务 */
void attitude_task(void *param)
{
    IMU_DATA_T imu = {0};
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    uint32_t print_divider = 0U;
    TickType_t preTime;//pretime作为RTOS里面专门记录时间戳的变量
    (void)param;
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
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    preTime = xTaskGetTickCount();
    for (;;)
      {
    float dt = 1.0f / (float)ATTITUDE_TASK_FREQ;

    if (ICM20948_Update(&imu, &hi2c1)) //IMU调用来进行姿态解算
    {
        /* 姿态解算 */
        Attitude_Update(imu.gyro.x,
                        imu.gyro.y,
                        imu.gyro.z,
                        imu.accel.x,
                        imu.accel.y,
                        imu.accel.z,
                        dt);
        Attitude_GetEuler(&roll,
                          &pitch,
                          &yaw);
			
//        Attitude_PID_Update(0.0f, 0.0f, 0.0f);
        taskENTER_CRITICAL();
			
        //姿态角赋值
        g_latest_roll  = roll;
        g_latest_pitch = pitch;
        g_latest_yaw   = yaw;
			  //角速度赋值
        g_latest_gx = imu.gyro.x;
        g_latest_gy = imu.gyro.y;
        g_latest_gz = imu.gyro.z;
				
        taskEXIT_CRITICAL();

        /* 10Hz output avoids saturating the blocking UART at 200Hz. */
        print_divider++;
        if (print_divider >= (ATTITUDE_TASK_FREQ / 10U))
        {
            print_divider = 0U;

            if (xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
            {
//							printf("Angle: R=%.3f P=%.3f Y=%.3f | "
//              "RateTarget: R=%.3f P=%.3f Y=%.3f\r\n",
//              roll,
//              pitch,
//              yaw,
//              RollRateTarget,
//              PitchRateTarget,
//              YawRateTarget);
//                printf("%.3f,%.3f,%.3f\r\n",
//                       roll,
//                       pitch,
//                       yaw);

                xSemaphoreGive(g_xMutex_Printf);
            }
        }
    }
    else
    {
        /* Skip this sample; the next loop retries the IMU read. */
    }

    vTaskDelayUntil(&preTime,
                    pdMS_TO_TICKS(1000.0f / ATTITUDE_TASK_FREQ));
}
}
