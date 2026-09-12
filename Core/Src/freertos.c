/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include <stdio.h>
#include "icm20948_reg.h"
#include "Bsp_imu.h"
#include "usart.h"
#include "math.h"
#include "attitude_task.h"
#include "Dshot.h"
#include "Control_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define userconfig_RTOS_DEBUG  0//是否开启调试任务
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdlib.h>

// VOFA+ 可调参数（初始为 0）
/* USER CODE END 0 */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,//osPriorityAboveNormal改变优先级了
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#if (1 ==userconfig_RTOS_DEBUG )
void RTOSDebugTask(void *param);
#endif //如果是1，执行调试RTOSDebugTask(void *param);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  //定时器TIM6用来给 getRunTimeCounterValue（）
	HAL_TIM_Base_Start(&htim6);
}

__weak unsigned long getRunTimeCounterValue(void)
{
  static unsigned long time = 0 ;
	static uint16_t lasttime = 0;
	static uint16_t nowtime = 0;
	
	nowtime = TIM6->CNT; //获得当前计数值
	
	//如果本次计数值小于上次计数值，说明发生了定时器计数溢出
	if( nowtime < lasttime )
	{
		time += (nowtime + 0xffff - lasttime); //溢出后的时间增量
	}		
	else time += ( nowtime - lasttime ) ; //未发生溢出，增量为本次时间-上次时间
	
	lasttime = nowtime;
	
	return time;
	//计算CPU的占用率
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
    //任务回调函数
}

/* USER CODE END 2 */

/* USER CODE BEGIN 4 */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	//检测是否发生栈溢出
	printf("%s stack overflow\r\n",pcTaskName);

}

/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	
  //内存分配失败函数
	 printf("malloc failed.check heapsize\r\n");
}

/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//创建Free RTOS调试任务
// #if ( 1 == userconfig_RTOS_DEBUG )
//	 static uint16_t delaytime = 5000;//打印时间间隔，单位tick
//	 xTaskCreate(RTOSDebugTask,"DebugTask",128, &delaytime ,osPriorityAboveNormal,NULL);
// #endif
	
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	xTaskCreate(attitude_task,"Attitude", 512, NULL, 3, NULL);
	xTaskCreate(control_task,"Control",512,NULL,2,NULL);
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
  /* USER CODE BEGIN StartDefaultTask */
void StartDefaultTask(void *argument)
{
	    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }                                                                                 
    /* USER CODE BEGIN StartDefaultTask */
    // 初始化传感器（只做一次）
//    while (!ICM20948_Init(&hi2c1)) {
//    }
//    // 然后这个任务什么都不做，让给 attitude_task 去算姿态
//    for (;;)
//    {
//    }
	  vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//#if ( 1 == userconfig_RTOS_DEBUG )
//void RTOSDebugTask(void *param)
//{
//	uint16_t* delaytime = (uint16_t*)param;
//	static char showbuf[500];
//	
//	while( 1 )
//	{
//		//打印CPU占比
//		vTaskGetRunTimeStats(showbuf);
//		printf("TaskName\tUseTime\tCPU\r\n");
//		printf("%s\r\n",showbuf);
//		vTaskDelay(*delaytime);

//		//打印剩余任务栈大小,单位word
//		vTaskList(showbuf);
//		printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
//		printf("%s\r\n",showbuf);
//		vTaskDelay(*delaytime);

//		

//		//打印剩余的堆区大小,单位bytes
//		printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
//		vTaskDelay(*delaytime);
//	}
//}
//#endif
/* USER CODE END Application */

