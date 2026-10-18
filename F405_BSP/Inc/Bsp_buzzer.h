#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H


#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
/*
 * 蜂鸣器接口,蜂鸣器高电平发出声响
 *
 * 使用方式：
 * UserBuzzer.on();      // 蜂鸣器打开
 * UserBuzzer.off();     // 蜂鸣器关闭
 * UserBuzzer.toggle();  // 蜂鸣器翻转
 */
/* 蜂鸣器接口 */
typedef struct
{
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);
} BuzzerInterface_t;

/* 蜂鸣器对象 */
extern BuzzerInterface_t UserBuzzer;

/* 成功、失败两个 FreeRTOS 软件定时器 */
extern TimerHandle_t g_BuzzerSuccessTimer;
extern TimerHandle_t g_BuzzerFailTimer;

void Timer_SuccessCallback(TimerHandle_t xTimer);//成功
void Timer_FailCallback(TimerHandle_t xTimer);//失败

/* 蜂鸣器初始化 */
void Buzzer_Init(void);
#endif /* __BSP_BUZZER_H */
