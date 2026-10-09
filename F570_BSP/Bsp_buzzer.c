#include "Bsp_buzzer.h"
#include "gpio.h"

static void BuzzerOn(void)
{
    HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,
                      UserBuzzer_Pin,
                      GPIO_PIN_SET);
}

static void BuzzerOff(void)
{
    HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,
                      UserBuzzer_Pin,
                      GPIO_PIN_RESET);
}

static void BuzzerToggle(void)
{
    HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,
                       UserBuzzer_Pin);
}

BuzzerInterface_t UserBuzzer = {
    .on = BuzzerOn,
    .off = BuzzerOff,
    .toggle = BuzzerToggle
};

/*========================================================
 * 成功提示 Timer 回调
 * 功能：
 * 蜂鸣器响 50ms，然后关闭
 *========================================================*/
 void Timer_SuccessCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    UserBuzzer.on();
    vTaskDelay(pdMS_TO_TICKS(50));
    UserBuzzer.off();
}


/*========================================================
 * 失败提示 Timer 回调
 * 功能：
 * 响 3 次
 *
 * ON  50ms
 * OFF 50ms
 * ON  50ms
 * OFF 50ms
 * ON  50ms
 * OFF 50ms
 *========================================================*/
 void Timer_FailCallback(TimerHandle_t xTimer)
{
    uint8_t i;
    (void)xTimer;
    for(i = 0; i < 3; i++)
    {
        UserBuzzer.on();
        vTaskDelay(pdMS_TO_TICKS(50));
        UserBuzzer.off();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
