/*这里的h文件使用到C++的内容，便于C++工程师也可以使用C文件*/
/*#ifdef _CPLUSPLUS
extern "C"{
#endif
void control_task(void *param);
#ifdef _CPLUSPLUS
}
#endif */
#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H
#ifdef __cplusplus
extern "C"{
#endif

#define CONTROL_TASK_FREQ 200  
void control_task(void *param);
#ifdef __cplusplus
}
#endif
#endif
