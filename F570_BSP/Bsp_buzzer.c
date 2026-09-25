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
