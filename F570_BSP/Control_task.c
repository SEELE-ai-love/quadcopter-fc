#include "Control_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "attitude_task.h"
#include "Dshot.h"

#define CONTROL_RASK_FREQ 200//保持200hz的频率

static pMotorInterface_t d_shot=&UserDshotMotor;//定义一个指针类型变量

/*typedef struct{
    uint8_t Telemetry;//遥测位
    short   throttle;//油门值
} dshotMotorVal_t;
*/
//电机启动之前是需要发送最低油门一段时间来解锁电机的
static dshotMotorVal_t m1={0,48};
static dshotMotorVal_t m2={0,48};
static dshotMotorVal_t m3={0,48};
static dshotMotorVal_t m4={0,48};

static uint32_t unlock_motor=0;//定义一个

void control_task(void*param)
{
	float roll,pitch,yaw;
	
	TickType_t pretime;
	(void)param;
	d_shot->init();//d_shot指向init
	pretime=xTaskGetTickCount();
	
	for(;;)
	{
		Attitude_GetLatest(&roll,&pitch,&yaw);
	//PID平衡预留接点
	//Mixer电机混控预留接点
	//光流计预留点
		if(unlock_motor<CONTROL_RASK_FREQ*4)
		{
			unlock_motor++;
		}//延时四秒，即
		else
		{
			//电机转动
//			m1.throttle=120;
//		  m2.throttle=120;
//			m3.throttle=120;
//			m4.throttle=120;
		}
		d_shot->set_target(m1,m2,m3,m4);//d_shot指向set_target
		vTaskDelayUntil(&pretime,pdMS_TO_TICKS(1000.0f/CONTROL_RASK_FREQ));
	}
}