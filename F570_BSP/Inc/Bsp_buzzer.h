#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

/*
 * 蜂鸣器接口,蜂鸣器高电平发出声响
 *
 * 使用方式：
 * UserBuzzer.on();      // 蜂鸣器打开
 * UserBuzzer.off();     // 蜂鸣器关闭
 * UserBuzzer.toggle();  // 蜂鸣器翻转
 */
typedef struct
{
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);

} BuzzerInterface_t, *pBuzzeInterface_t;


/* 对外提供蜂鸣器对象 */
extern BuzzerInterface_t UserBuzzer;

#endif /* __BSP_BUZZER_H */
