#include "usart.h"

//接入Free RTOS
#include "FreeRTOS.h"
#include "tim.h"
#include "Bsp_stp23L.h"
#include <stdio.h>
#include "queue.h"

extern uint8_t uart4_recv;
extern uint8_t uart1_recv;
//软件复位进Boot，实现自动复位
static void _System_Reset_(uint8_t uart_recv)
{
	extern void ResetSystem(uint8_t isFromISR);
	
	static uint8_t res_buf[5];
	static uint8_t res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//进行软件复位，复位后执行 BootLoader 程序
			//ResetSystem(1);//进行软件复位，复位后执行 BootLoader 程序
		}
	}
}


//串口接收中断函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if( huart == &huart4 ) //蓝牙使用的串口
	{
		//复位BootLoadr检测指令
		_System_Reset_(uart4_recv);
		
		HAL_UART_Receive_IT(&huart4,&uart4_recv,1);
	}
	else if( huart == &huart1 )
	{
		_System_Reset_(uart1_recv);
		HAL_UART_Receive_IT(&huart1,&uart1_recv,1);
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN)会启动两种中断源，一个是UART 空闲中断（IDLE），
//  另一个是DMA 传输完成中断（TC），所以Size的结果才是一个195，另一个200/2=100
// ============================================================
// ============================================================
// 激光模块的有关参数：
// QueueHandle_t g_xQueueSTP23L_Ori   //激光模块的队列句柄
// DMABuf_oridata_stp23L //存放DMA搬运的光流模块的数据
// userconfig_STP23L_DMABUF_LEN //DMA缓冲区,200
// xHigherPriorityTaskWoken //判断是否出现更高优先级的任务
// xQueueSendFromISR(),是队列的中断版本
// ============================================================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)//中断回调函数
{
	//Size表示存放数据的数目
	BaseType_t xHigherPriorityTaskWoken =pdFALSE;//去检测有没有更高优先级的任务
	if(huart ==&huart5 &&HAL_UART_STATE_READY ==huart->RxState)
	{
		extern 	QueueHandle_t g_xQueueSTP23L_Ori;
		DMABuf_oridata_stp23L.DataLen =Size;
		   xQueueSendFromISR(g_xQueueSTP23L_Ori, &DMABuf_oridata_stp23L,
                          &xHigherPriorityTaskWoken);
	//重新启动DMA搬运，因为用的是DMA的normal模式
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	}
	//如果发生更高优先级的任务，在这里进行调度（如果更高优先级的任务被唤醒，那么就转到更高的优先级任务去）
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
