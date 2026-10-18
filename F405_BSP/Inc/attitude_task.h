#ifndef __ATTITUDE_TASK_H
#define __ATTITUDE_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"

//姿态、角速度、加速度放在同一个快照里面，避免读到不同轮的数据
typedef struct
{
    float roll,pitch,yaw;
    float gx,gy,gz;
    float ax,ay,az;
    TickType_t sample_tick;
    uint32_t sequence;
} AttitudeSnapshot_t;

// IMU_FIFO BEGIN：软件环形FIFO，保存已经成功读取并解算的一轮IMU
#define ATTITUDE_FIFO_CAPACITY 16U

typedef struct
{
    uint32_t pushed;//成功放入的总帧数
    uint32_t popped;//Control已经取出的总帧数
    uint32_t overflow;//满了以后丢掉的最老帧数
    uint32_t pending;//当前还有多少帧没取
    uint32_t high_water;//曾经最多积压多少帧
} AttitudeFifoStats_t;
// IMU_FIFO END

#ifdef __cplusplus
extern "C" {
#endif

/* ?? 【补充】attitude_task.c 里用到这个宏，但原来没定义，编译会报错 */
#define ATTITUDE_TASK_FREQ  200

/* ?? 任务入口 */
void attitude_task(void *param);
uint8_t Attitude_GetSnapshot(AttitudeSnapshot_t *out);

// IMU_FIFO BEGIN：Pop只有Control里面的位置估计器调用，其他任务仍然读最新快照
uint8_t Attitude_FifoPop(AttitudeSnapshot_t *out);
void Attitude_FifoGetStats(AttitudeFifoStats_t *out);
// IMU_FIFO END

void Attitude_GetAccel(float *ax,float *ay,float *az);// get accel m/s^2

/* ?? 获取当前最新姿态角 */
/* ?? 跨任务接口：带临界区保护 */
void Attitude_GetLatest(float *roll, float *pitch, float *yaw);//输出姿态角，姿态解算要用
void Attitude_GetGyro(float *gx,float *gy,float *gz);//输出角速度，PID要用

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_TASK_H */
