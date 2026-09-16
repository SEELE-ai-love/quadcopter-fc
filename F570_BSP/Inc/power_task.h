#ifndef __POWER_TASK_H
#define __POWER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 电源监测任务频率：不需要跟姿态解算一样快，10Hz足够看电压趋势 */
#define POWER_TASK_FREQ  10 //10Hz

void power_task(void *param);

/* 跟Attitude_GetLatest同样的设计模式：其他任务不要直接读ADC，
 * 统一通过这个接口拿最新的电压/电流值 */
 
void Power_GetLatest(float *voltage, float *current);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_TASK_H */