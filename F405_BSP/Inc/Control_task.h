/*这里的h文件使用到C++的内容，便于C++工程师也可以使用C文件*/
/*#ifdef _CPLUSPLUS
extern "C"{
#endif
void control_task(void *param);
#ifdef _CPLUSPLUS
}
#endif */
#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#define CONTROL_TASK_FREQ 200U
/* 控制来源 */
enum
{
    IDLECmd = 0,    /* 控制空闲 */
    APPCmd  = 1,    /* APP控制 */
    PS2Cmd  = 2     /* PS2手柄控制 */
};

/* 四轴控制结构体 */
typedef struct
{
    uint8_t source; /* 控制来源 */

    float pitch;    /* 前后控制，+前进，-后退 */
    float roll;     /* 左右控制，+左移，-右移 */
    float gyroz;    /* Yaw控制，+逆时针，-顺时针 */
    float height;   /* 高度控制 */

} FlyControlType_t;

/* 控制队列 */
extern QueueHandle_t g_xQueueFlyControl;

/* 向控制队列写入控制指令 */
void WriteFlyControlQueue(FlyControlType_t val,
                          uint8_t writeEnv,
                          BaseType_t *HigherPriorityTask);

void control_task(void *param);

void Control_GetDebugData(float *m1,
                          float *m2,
                          float *m3,
                          float *m4,
                          float *base_throttle);
#ifdef __cplusplus
}
#endif

#endif
