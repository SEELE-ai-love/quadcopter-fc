#include "Bsp_adc.h"
#include "adc.h"

float g_robotVOL = 12.0f;//12V的电压
#define userconfig_ADCDMA_BUF_LEN 80 //设置dma搬运通道的个数
 
static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };//存放ADC数据的二维数组，总共
 
void ADC_Userconfig_Init(void)
{
	//启动ADC采集并设置DMA传输，这一步之后ADC+DMA会在硬件层面持续循环采样，
	//不需要任何任务主动去"驱动"它，只需要调用一次
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
}
 
//获取DMA搬运后的ADC值（取滑动平均值，抑制采样噪声）
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;
	//channel==2 会越界访问 g_Adc1Buf[i][2]，改成 >= 2 才能正确拦住非法索引
	if( channel >= 2 ) return 0;
	
	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];//叠加80次ADC采样的值
	}
	return tmp/userconfig_ADCDMA_BUF_LEN;//取80的次的平均值
}
 
ADCInterface_t UserADC1 = {
	.init = ADC_Userconfig_Init,
	.getValue = USER_ADC_Get_AdcBufValue,
};//驱动挂载
