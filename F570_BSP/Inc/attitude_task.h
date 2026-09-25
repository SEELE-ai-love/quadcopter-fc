#ifndef __ATTITUDE_TASK_H
#define __ATTITUDE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* ?? 【补充】attitude_task.c 里用到这个宏，但原来没定义，编译会报错 */
#define ATTITUDE_TASK_FREQ  200

/* ?? 任务入口 */
void attitude_task(void *param);

/* ?? 获取当前最新姿态角 */
/* ?? 跨任务接口：带临界区保护 */
void Attitude_GetLatest(float *roll, float *pitch, float *yaw);//输出姿态角，姿态解算要用
void Attitude_GetGyro(float *gx,float *gy,float *gz);//输出角速度，PID要用

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_TASK_H */
