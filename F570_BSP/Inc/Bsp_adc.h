#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

#define userconfigADC_VBAT_CHANNEL  0//电压
#define userconfigADC_Curr_CHANNEL  1//电流

typedef struct {
	void (*init)(void);
    uint16_t (*getValue)(uint8_t channel);
}ADCInterface_t,*pADCInterface_t;//与挂载结合在一块的结构体指针

uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel);//ADC调用函数
void ADC_Userconfig_Init(void);//init函数
extern float g_robotVOL;//全局变量-电池电压

extern ADCInterface_t UserADC1;//驱动挂载内容
#endif 
