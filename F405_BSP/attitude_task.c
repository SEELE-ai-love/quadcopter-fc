// ============================================================
// 由单任务耦合解耦成多任�?+ FreeRTOS 形式
// ============================================================

#include "attitude_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdio.h>

#include "Bsp_imu.h"
#include "Bsp_iic.h"
#include "usart.h"
#include "icm20948_reg.h"
#include "attitude_algorithm.h"
#include "pid.h"
#include "math.h"
#define GYRO_CALIBRATE_NUM 2000
/* ============================================================
 * 共享姿态数�?
 * ============================================================ */
static float g_latest_roll  = 0.0f;
static float g_latest_pitch = 0.0f;
static float g_latest_yaw   = 0.0f;

static float g_latest_gx = 0.0f;
static float g_latest_gy = 0.0f;
static float g_latest_gz = 0.0f;

static float g_latest_ax = 0.0f;
static float g_latest_ay = 0.0f;
static float g_latest_az = 0.0f;
static TickType_t g_latest_sample_tick = 0U;
static uint32_t g_latest_sequence = 0U;

// IMU_FIFO BEGIN���̶����飬����������RTOS�ѣ�Ҳ���ı�����200Hz����
static AttitudeSnapshot_t g_attitude_fifo[ATTITUDE_FIFO_CAPACITY];
static uint16_t g_attitude_fifo_read = 0U;
static uint16_t g_attitude_fifo_write = 0U;
static AttitudeFifoStats_t g_attitude_fifo_stats = {0};

//����ֻ���ڷ������յĶ��ٽ����ڵ��ã���Ҫ����һ���ٽ���
static void Attitude_FifoPushLocked(void)
{
    AttitudeSnapshot_t *sample = &g_attitude_fifo[g_attitude_fifo_write];

    if(g_attitude_fifo_stats.pending == ATTITUDE_FIFO_CAPACITY)
    {
        //���˱������µ����ݣ���������һ֡�������߻�����sequence�Ƿ�����
        g_attitude_fifo_read = (uint16_t)((g_attitude_fifo_read+1U) % ATTITUDE_FIFO_CAPACITY);
        g_attitude_fifo_stats.overflow++;
    }
    else
    {
        g_attitude_fifo_stats.pending++;
    }

    sample->roll = g_latest_roll;
    sample->pitch = g_latest_pitch;
    sample->yaw = g_latest_yaw;
    sample->gx = g_latest_gx;
    sample->gy = g_latest_gy;
    sample->gz = g_latest_gz;
    sample->ax = g_latest_ax;
    sample->ay = g_latest_ay;
    sample->az = g_latest_az;
    sample->sample_tick = g_latest_sample_tick;
    sample->sequence = g_latest_sequence;

    g_attitude_fifo_write = (uint16_t)((g_attitude_fifo_write+1U) % ATTITUDE_FIFO_CAPACITY);
    g_attitude_fifo_stats.pushed++;
    if(g_attitude_fifo_stats.pending > g_attitude_fifo_stats.high_water)
    {
        g_attitude_fifo_stats.high_water = g_attitude_fifo_stats.pending;
    }
}

uint8_t Attitude_FifoPop(AttitudeSnapshot_t *out)
{
    uint8_t available = 0U;

    if(out == NULL)
    {
        return 0U;
    }

    //ÿ��ֻ����һ֡�������ٽ�������㡢���ֻ��ߴ�ӡ
    taskENTER_CRITICAL();
    if(g_attitude_fifo_stats.pending != 0U)
    {
        *out = g_attitude_fifo[g_attitude_fifo_read];
        g_attitude_fifo_read = (uint16_t)((g_attitude_fifo_read+1U) % ATTITUDE_FIFO_CAPACITY);
        g_attitude_fifo_stats.pending--;
        g_attitude_fifo_stats.popped++;
        available = 1U;
    }
    taskEXIT_CRITICAL();

    return available;
}

void Attitude_FifoGetStats(AttitudeFifoStats_t *out)
{
    if(out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = g_attitude_fifo_stats;
    taskEXIT_CRITICAL();
}
// IMU_FIFO END

float gyro_sum_x = 0.0f;
float gyro_sum_y = 0.0f;
float gyro_sum_z = 0.0f;

/* ============================================================
 * printf 互斥�?
 * ============================================================ */
extern SemaphoreHandle_t g_xMutex_Printf;


/* ============================================================
 * 跨任务接口：姿态角
 * ============================================================ */
uint8_t Attitude_GetSnapshot(AttitudeSnapshot_t *out)
{
    if(out == NULL)
    {
        return 0U;
    }
    //����ֻ����һ��IMU�����ݣ������ٽ����������
    taskENTER_CRITICAL();
    out->roll = g_latest_roll;
    out->pitch = g_latest_pitch;
    out->yaw = g_latest_yaw;
    out->gx = g_latest_gx;
    out->gy = g_latest_gy;
    out->gz = g_latest_gz;
    out->ax = g_latest_ax;
    out->ay = g_latest_ay;
    out->az = g_latest_az;
    out->sample_tick = g_latest_sample_tick;
    out->sequence = g_latest_sequence;
    taskEXIT_CRITICAL();
    return (out->sequence != 0U) ? 1U : 0U;
}

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

void Attitude_GetAccel(float *ax,
                       float *ay,
                       float *az)
{
    taskENTER_CRITICAL();

    *ax = g_latest_ax;
    *ay = g_latest_ay;
    *az = g_latest_az;

    taskEXIT_CRITICAL();
}


/* ============================================================
 * 姿态任�?
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
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)"BOOT:ATT_TASK\r\n", 15U, 100U);


    /* ========================================================
     * 姿态算法选择
     *
     * Mahony / EKF 二选一
     * ======================================================== */
    //Attitude_Init(ATTITUDE_ALGO_MAHONY);
    Attitude_Init(ATTITUDE_ALGO_EKF);


    /* ========================================================
     * 初始�?ICM20948
     * ======================================================== */
    while (!ICM20948_Init(&hi2c1))
    {
        (void)HAL_UART_Transmit(&huart1, (const uint8_t *)"BOOT:IMU_WAIT\r\n", 15U, 100U);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)"BOOT:IMU_OK\r\n", 13U, 100U);


    /* ========================================================
     * 获取任务起始时间
     * ======================================================== */
    preTime = xTaskGetTickCount();


    /* ========================================================
     * 主循�?
     * ======================================================== */
    for (;;)
    {
        float dt = 1.0f / (float)ATTITUDE_TASK_FREQ;


        /* ====================================================
         * 读取 IMU
         * ==================================================== */
        if (ICM20948_Update(&imu, &hi2c1))
        {
            // IMU_FIFO����¼�������ʱ�䣬���ܰѳ�FIFO��ʱ�䵱�ɲ���ʱ��
            TickType_t sample_tick = xTaskGetTickCount();

            /* =================================================
             * 姿态解�?
             *
             * gyro：rad/s
             * accel：当�?Bsp_imu.c 的单�?
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
             * 这里已经经过�?
             *
             * 原始姿�?
             *     �?
             * AttitudeZero
             *     �?
             * 最终姿�?
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
             * 注意�?
             * 这里保留你原来的 Roll / Pitch 方向定义
             * ================================================= */
            //�ɹ������󡢷���ǰ������ʱ���ǣ�������оƬӲ��ͬ��
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

            g_latest_ax = imu.accel.x;
            g_latest_ay = imu.accel.y;
            g_latest_az = imu.accel.z;
            g_latest_sample_tick = sample_tick;
            g_latest_sequence++;
            // IMU_FIFO��ͬһ�ֿ��շ���FIFO������ֻ���ƣ���������
            Attitude_FifoPushLocked();

            taskEXIT_CRITICAL();


            /* =================================================
             * 串口打印
             *
             * 200Hz / 20 = 10Hz
             * 每秒打印10�?
             *
             * %.1f�?
             * 保留 0.1°
             * ================================================= */
            print_divider++;
     /* =================================================
             * 串口打印
             *
             * 验证最基本的单位是否正确，串级PID统一成弧度制
             * gyro的rad\s和dps是否成功统一
             * accel加速度是否是m\s²
             * ACC_NORM �?9.8 m/s²，说命加速度的缩放没有问�?
             * 
       * ================================================= */
// static uint8_t print_cnt = 0;

// print_cnt++;

// if(print_cnt >= 40)   // 200Hz / 40 = 5Hz
// {
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
// }
        //    if (print_divider >=
        //        (ATTITUDE_TASK_FREQ / 10U))
        //    {
        //        print_divider = 0U;

        //        if (xSemaphoreTake(g_xMutex_Printf,
        //                           portMAX_DELAY) == pdTRUE)
        //        {
        //            printf("Angle: R=%+.3f P=%+.3f Y=%+.3f | "
        //                   "gyro: x=%+.3f y=%+.3f z=%+.3f | "
        //                   "READY=%d\r\n",
        //                   roll,
        //                   pitch,
        //                   yaw,
        //                   imu.gyro.x,
        //                   imu.gyro.y,
        //                   imu.gyro.z,
        //                   Attitude_IsZeroReady());

        //            xSemaphoreGive(g_xMutex_Printf);
        //        }
        //    }
        }
        //IMU��ȡʧ��ҲҪ�ó�CPU�����ܰѸ����ȼ������ܳ���ѭ��
		//每一轮都要走到下面的延时，不能在这里提前结束for循环


        /* ====================================================
         * 保持 200 Hz
         * ==================================================== */
        vTaskDelayUntil(&preTime,
                        pdMS_TO_TICKS(
                        1000.0f /
                        (float)ATTITUDE_TASK_FREQ));
    }
}
// }
