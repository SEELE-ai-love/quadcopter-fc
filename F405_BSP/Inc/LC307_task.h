/* LC307_task.h */
#ifndef LC307_TASK_H
#define LC307_TASK_H
#include <stdint.h>
#ifdef   __cplusplus
extern "C"{
#endif



void LC307_task(void *param);
//只由200Hz的Control任务调用，IMU预测和位置积分都在这里完成
void LC307_EstimatorUpdate(void);
typedef struct
{
    float x_pos;      /* 相对位置，m */
    float y_pos;

    float x_speed;    /* 融合速度，m/s */
    float y_speed;
} FlyPosMsg_t;

//光流估计快照的状态，位置控制只读取这些信息，不操作串口
typedef struct
{
    uint32_t sequence;//每发布一个有效完整帧，序号加1
    uint32_t fault_epoch;//光流任务发现失效的代数
    uint32_t rx_epoch;//串口接收故障代数
    uint32_t age_ms;//距离当前快照接收时刻的时间
    uint32_t receive_ms;//同一DMA块里的多帧保留相同的接收时间
} LC307_Status_t;

//sequence是估计器已消费的最新帧序号，可以跳号；200Hz预测不增加它
uint8_t LC307_GetLatestEx(FlyPosMsg_t *out,LC307_Status_t *status);

/* 任务上下文调用；返回 0 时 out 清零，NULL 参数直接返回 0。 */
uint8_t LC307_GetLatest(FlyPosMsg_t *out);
#ifdef __cplusplus
}
#endif
#endif