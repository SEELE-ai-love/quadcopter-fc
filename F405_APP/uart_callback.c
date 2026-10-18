#include "usart.h"

//接入Free RTOS
#include "FreeRTOS.h"
#include "tim.h"
#include "Bsp_stp23L.h"
#include "Bsp_lc307.h"
#include <stdio.h>
#include "queue.h"
#include "task.h"

volatile uint32_t g_lc307_queue_overflow_count = 0U;

volatile uint32_t g_lc307_rx_epoch = 0U;
volatile uint8_t g_lc307_rx_requests = 0U;

static void LC307_RxFaultFromISR(uint8_t need_restart)
{
    UBaseType_t saved_mask;
    saved_mask = taskENTER_CRITICAL_FROM_ISR();
    g_lc307_rx_epoch++;
    if(need_restart != 0U)
    {
        g_lc307_rx_requests |= LC307_RX_REQUEST_RECOVER;
    }
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
}

static void LC307_RequestStartFromISR(void)
{
    UBaseType_t saved_mask;
    saved_mask = taskENTER_CRITICAL_FROM_ISR();
    g_lc307_rx_requests |= LC307_RX_REQUEST_START;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
}


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
		extern QueueHandle_t g_xQueueBlueTooth_Ori;
		//复位BootLoadr检测指令
		_System_Reset_(uart4_recv);
		
	  //把接收到的字符写入队列
		xQueueSendFromISR(g_xQueueBlueTooth_Ori, &uart4_recv,
                          &xHigherPriorityTaskWoken);
		
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
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN)会启动两种中断源，一个是UART 空闲中断（IDLE），另一个是DMA 传输完成中断（DMA_TC）
/*
 * ReceiveToIdle DMA 可能产生三种事件：
 * HT：半传输，200 字节缓冲区对应 Size = 100，接收仍继续。
 * TC：传输完成，200 字节缓冲区对应 Size = 200。
 * IDLE：收到数据后串口空闲，例如本次收到195字节。
 *
 * Normal 模式：HT 不结束接收；TC/IDLE 会结束本次接收。
 * 当前通过 RxState == READY，只处理接收结束后的数据。
 */
// ============================================================

// ============================================================
// 激光模块的有关参数：
// QueueHandle_t g_xQueueSTP23L_Ori   //激光模块的队列句柄
// DMABuf_oridata_stp23L //存放DMA搬运的光流模块的数据
// userconfig_STP23L_DMABUF_LEN //DMA缓冲区,200
// xHigherPriorityTaskWoken //判断是否出现更高优先级的任务
// xQueueSendFromISR()//是队列的中断版本
// ============================================================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)//中断回调函数
{
	//Size表示存放数据的数目
	BaseType_t xHigherPriorityTaskWoken =pdFALSE;//去检测有没有更高优先级的任务
	if(huart ==&huart5 &&HAL_UART_STATE_READY ==huart->RxState)
	{
		extern 	QueueHandle_t g_xQueueSTP23L_Ori;
		DMABuf_oridata_stp23L.DataLen =Size;
		xQueueSendFromISR(g_xQueueSTP23L_Ori, &DMABuf_oridata_stp23L, &xHigherPriorityTaskWoken);
		
	  //重新启动DMA搬运，因为用的是DMA的normal模式
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	}
	else if(huart ==&huart3)
	{
		extern 	QueueHandle_t g_xQueueLC307_Ori;
		HAL_UART_RxEventTypeTypeDef event_type;

		event_type = HAL_UARTEx_GetRxEventType(huart);
		//HT不结束接收，所以不复制半块数据，也不提出DMA重启请求
		if(event_type == HAL_UART_RXEVENT_HT)
		{
			return;
		}
		if(huart->hdmarx == NULL || huart->hdmarx->Init.Mode != DMA_NORMAL)
		{
			return;
		}
		if(event_type != HAL_UART_RXEVENT_IDLE && event_type != HAL_UART_RXEVENT_TC)
		{
			return;
		}
		if(huart->RxState != HAL_UART_STATE_READY)
		{
			return;
		}

		if(Size == 0U || Size > sizeof(DMABuf_oridata_LC307.Buf))
		{
			LC307_RxFaultFromISR(0U);
			LC307_RequestStartFromISR();
			return;
		}
		DMABuf_oridata_LC307.DataLen=Size;
		DMABuf_oridata_LC307.RxTick = xTaskGetTickCountFromISR();
		DMABuf_oridata_LC307.RxEpoch = g_lc307_rx_epoch;
		if (xQueueSendFromISR(g_xQueueLC307_Ori,
		                     &DMABuf_oridata_LC307,
		                     &xHigherPriorityTaskWoken) != pdPASS)
		{
			g_lc307_queue_overflow_count++;
			LC307_RxFaultFromISR(0U);
		}
		
	  //重新启动DMA搬运，因为用的是DMA的normal模式
	  //补充：这里只提出请求，中断不再调用StartReceive或AbortReceive
	  LC307_RequestStartFromISR();
	}
	//如果发生更高优先级的任务，在这里进行调度（如果更高优先级的任务被唤醒，那么就转到更高的优先级任务去）
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//串口错误回调：只标记故障，接收恢复交给光流任务
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
    {
        LC307_RxFaultFromISR(1U);
    }
}
