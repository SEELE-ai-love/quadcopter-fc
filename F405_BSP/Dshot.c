#include "tim.h"
#include "Dshot.h"

#define Dshot_BIT_1 420
#define Dshot_BIT_0 210
#define DSHOT_BUF_LEN 20

static uint16_t dshot_buf[4][DSHOT_BUF_LEN]={0};//电机的二维数组缓冲区

static TIM_HandleTypeDef*dshot_tim=&htim8;//选定TIM8定时器

static uint16_t dshot_crc(dshotMotorVal_t val)//11位电机位+1位遥测位+4位CRC位拼接
{
	uint16_t packet=((val.throttle)<<1)|(val.Telemetry?1:0);
	uint8_t crc=(packet ^(packet>>4)^(packet>>8))&0x0F;
	return (packet<<4)|crc;
}

static void pwmWriteDigital(dshotMotorVal_t m1,dshotMotorVal_t m2,dshotMotorVal_t m3,dshotMotorVal_t m4)
{
	m1.throttle=(m1.throttle>2047)?2047:m1.throttle;
	m2.throttle=(m2.throttle>2047)?2047:m2.throttle;
	m3.throttle=(m3.throttle>2047)?2047:m3.throttle;
	m4.throttle=(m4.throttle>2047)?2047:m4.throttle;
	
	m1.throttle=dshot_crc(m1);
	m2.throttle=dshot_crc(m2);
	m3.throttle=dshot_crc(m3);
	m4.throttle=dshot_crc(m4);
	for(uint8_t i=0;i<16;i++)
	{
		dshot_buf[0][i]=(m1.throttle>>(15-i)&0x01)?Dshot_BIT_1:Dshot_BIT_0;
		dshot_buf[1][i]=(m2.throttle>>(15-i)&0x01)?Dshot_BIT_1:Dshot_BIT_0;
		dshot_buf[2][i]=(m3.throttle>>(15-i)&0x01)?Dshot_BIT_1:Dshot_BIT_0;
		dshot_buf[3][i]=(m4.throttle>>(15-i)&0x01)?Dshot_BIT_1:Dshot_BIT_0;
	}
	
	HAL_TIM_PWM_Start_DMA(dshot_tim,TIM_CHANNEL_1,(uint32_t*)&dshot_buf[0][0],DSHOT_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(dshot_tim,TIM_CHANNEL_2,(uint32_t*)&dshot_buf[1][0],DSHOT_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(dshot_tim,TIM_CHANNEL_3,(uint32_t*)&dshot_buf[2][0],DSHOT_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(dshot_tim,TIM_CHANNEL_4,(uint32_t*)&dshot_buf[3][0],DSHOT_BUF_LEN);
}

static void motor_init(void)
{
	//空的
}

//UserDshotMotor包含两个结构，.init和.set_target
MotorInterface_t UserDshotMotor =
{
    .init = motor_init,
    .set_target = pwmWriteDigital,
};//驱动挂载
