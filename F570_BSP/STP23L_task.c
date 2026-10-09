#include "FreeRTOS.h" 
#include "queue.h"
#include <stdio.h>
#include "Bsp_stp23L.h"
#include "STP23L_task.h"
#include "semphr.h"


static OriData_STP23L_t queue_recv;

//========== 互斥锁 ==========//
extern SemaphoreHandle_t g_xMutex_Printf;  

void STP23L_task(void *param)
{
	
	extern 	QueueHandle_t g_xQueueSTP23L_Ori;//声明激光模块的队列
	while(1)
	{
	 xQueueReceive(g_xQueueSTP23L_Ori,&queue_recv,portMAX_DELAY);
		//因为此工程的UART串口通信采用的是逐字节的状态机，所以这里需要用单字节循环的形式来进行传递
		for(uint16_t i=0;i<queue_recv.DataLen;i++)
		{
	 //xQueueReceive表示接收队列，这里的延时用portMAX_DELAY表示如果没有接收到队列，则会阻塞这个任务
	 //处理激光模块信息
		stp23L_getdistance_callback(queue_recv.Buf[i],&g_readonly_distance);
		}
		if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
    {
		//printf("%.3f\r\n",g_readonly_distance);
		xSemaphoreGive(g_xMutex_Printf); 
    }
	}
}
