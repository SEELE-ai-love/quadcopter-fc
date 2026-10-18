/* 关于光流位置环的接口 */
#ifndef POSITION_HOLD_H
#define POSITION_HOLD_H

#include <stdint.h>
#include "LC307_task.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef struct
{
    float kp_x;          //位置比例项，单位rad/m
    float kp_y;
    float kv_x;          //速度阻尼项，单位rad/(m/s)
    float kv_y;
    float angle_limit;   //位置环给出的最大倾角，单位rad
} PositionHoldConfig_t;

typedef struct
{
    float control_x;     // PD_CONTROL 向右为正的控制需求，单位rad
    float control_y;     //向前为正的控制需求，单位rad
    float roll_add;
    float pitch_add;
} PositionHoldOutput_t;

/*这里只计算位置PD，不操作串口、电机，也不复位姿态PID*/
uint8_t PositionHold_Compute(const FlyPosMsg_t *estimate,
                             float target_x,
                             float target_y,
                             const PositionHoldConfig_t *config,
                             PositionHoldOutput_t *output);

#ifdef __cplusplus
}
#endif
#endif
