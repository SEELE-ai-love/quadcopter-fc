#ifndef __TASK_H
#define __TASK_H

#include "Bsp_imu.h"
void Attitude_GetLatest(float *roll,float *pitch,float *yaw);
void attitude_task(void*param);
#define ATTITUDE_TASK_FREQ  200

#endif
