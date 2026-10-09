#include <stdio.h> 

#include "FreeRTOS.h" 
#include "task.h" 
#include "queue.h" 

#include "pid.h" 

#include "Bsp_buzzer.h"
#include "timers.h"
//Qt蓝牙上位机实现实时调参
void Bluetooth_task(void *param) 
{ 
	extern QueueHandle_t g_xQueueBlueTooth_Ori; 
	
	//接收蓝牙队列数据 
	uint8_t bluetooth_recv=0; 
	
	//上一次数据内容 
	uint8_t last_recv=0; 
	
	//用于PID参数组的接收 
	#define pidparam_len 50 
	char pid_param[pidparam_len]; 
	uint8_t pid_cnt =0; 
	uint8_t state_sm=0; 
	uint8_t ready=0; 
	
	//用于PID参数解析 
	uint8_t param_type; 
	float kp,ki,kd; 
	
	//pid参数格式示例： {S0:1.0,-2.1,15}   //设置数据组0，kp=1.0，ki=-2.1，kd=15 
	
	while(1) 
	{ 
		xQueueReceive(g_xQueueBlueTooth_Ori,&bluetooth_recv,portMAX_DELAY );//等待队列数据 
		
		//PID参数接收状态机
		switch(state_sm ) 
		{ 
			case 0: 
				if( last_recv =='{'&& bluetooth_recv =='S') 
				{ 
					pid_param[0]='{'; 
					pid_param[1]='S'; 
					pid_cnt=2; 
					state_sm=1; 
				} 
				break; 
				
			case 1: 
				//接收pid字符数据
				pid_param[pid_cnt++] = bluetooth_recv;
				if( pid_cnt >= pidparam_len-2 )
				{
					//超出缓冲区, 弃用本次数据
					pid_cnt=0;
					state_sm=0;
				}
				else if( bluetooth_recv == '}' )
				{
					//开始解析，添加字符串结束符
					pid_param[pid_cnt] = '\0';
					ready = 1;
					state_sm = 0;	//重新开始接收
					pid_cnt = 0;
				}
				break;
				
			default:
				break;
		}
		
		//==================== 数据赋值解析部分 ====================
		if( ready )
		{
			ready = 0;
			uint8_t res = sscanf(pid_param,"{S%c:%f,%f,%f}", &param_type, &kp, &ki, &kd);
			if( res == 4 )	//成功解析到4个变量
			{
				//蜂鸣器提示发送成功（需要自己实现蜂鸣器函数）
				xTimerStart(g_BuzzerSuccessTimer, 0);
				param_type = param_type - '0';	//字符'0'~'7'转数字0~7
				//验证飞控板和电脑蓝牙软件是否成功连接（蓝牙软件调参，串口打印数据）
			printf("type:%d\r\n", param_type);
      printf("kp:%f\r\n", kp);
      printf("ki:%f\r\n", ki);
      printf("kd:%f\r\n", kd);
				switch(param_type)
				{
					case 0:
						//roll-rate内环
						RollRatePID.kp = kp; RollRatePID.ki = ki; RollRatePID.kd = kd;
						break;
					case 1:
						//roll外环
						RollPID.kp = kp; RollPID.ki = ki; RollPID.kd = kd;
						break;
					case 2:
						//pitch-rate内环
						PitchRatePID.kp = kp; PitchRatePID.ki = ki; PitchRatePID.kd = kd;
						break;
					case 3:
						//pitch外环
						PitchPID.kp = kp; PitchPID.ki = ki; PitchPID.kd = kd;
						break;
					case 4:
						//yaw-rate内环
						YawRatePID.kp = kp; YawRatePID.ki = ki; YawRatePID.kd = kd;
						break;
					case 5:
						//yaw外环
						YawPID.kp = kp; YawPID.ki = ki; YawPID.kd = kd;
						break;
//					case 6:
//						//height-rate内环
//						HeightratePID.kp = kp; HeightratePID.ki = ki; HeightratePID.kd = kd;
//						break;
//					case 7:
//						//height外环
//						HeightPID.kp = kp; HeightPID.ki = ki; HeightPID.kd = kd;
//						break;
					default:
						break;
				}
			}
		}
		
		//保存本次接收字符，给下一轮循环判断
		last_recv = bluetooth_recv;
	}
}
