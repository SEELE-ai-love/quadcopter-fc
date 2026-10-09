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
#include "timers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include "icm20948_reg.h"
#include "Bsp_imu.h"
#include "usart.h"
#include "math.h"
#include "attitude_task.h"
#include "Dshot.h"
#include "Control_task.h"
#include "power_task.h"
#include "Bsp_stp23L.h"
#include "STP23L_task.h"
#include "usb_host.h"
#include "show_task.h"
#include "Bluetooth_task.h"
#include "Bsp_buzzer.h"
#include "PS2_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//记住整个Free RTOS的框图：
//MX_FREERTOS_Init()
//│
//├── 创建 Mutex 互斥锁
//│
//├── 创建 Semaphore 信号量
//│
//├── 创建 Timer Free RTOS软件定时器
//│
//├── 创建 Queue 队列
//│ 
//├── 创建 Task 任务
//│
//└── 创建 Event Group 事件组
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

//队列创建
 QueueHandle_t g_xQueueSTP23L_Ori=NULL;//激光雷达队列任务句柄
 QueueHandle_t g_xQueueBlueTooth_Ori = NULL;//蓝牙软件队列任务句柄
 QueueHandle_t g_xQueueFlyControl; //PS手柄队列任务句柄
//互斥锁创建
 SemaphoreHandle_t g_xMutex_Printf = NULL;//printf互斥锁句柄
//定时器创建
 TimerHandle_t g_BuzzerSuccessTimer;
 TimerHandle_t g_BuzzerFailTimer;
 
#include <string.h>
#include <stdlib.h>
/* USER CODE END 0 */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityIdle,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#if (1 ==userconfig_RTOS_DEBUG )
void RTOSDebugTask(void *param);
#endif //如果是1，执行调试RTOSDebugTask(void *param);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
extern UART_HandleTypeDef *DebugSerial;
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

static void DebugUart_RawWrite(const char *text)
{
  if ((DebugSerial != NULL) && (text != NULL))
  {
    (void)HAL_UART_Transmit(DebugSerial,
                            (const uint8_t *)text,
                            (uint16_t)strlen(text),
                            5U);
  }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  (void)xTask;
  DebugUart_RawWrite("stack overflow: ");
  DebugUart_RawWrite((const char *)pcTaskName);
  DebugUart_RawWrite("\r\n");

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
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
  DebugUart_RawWrite("malloc failed. check heap size\r\n");

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
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
	
	 // ========== 在这里创建 printf 互斥锁 ==========
  g_xMutex_Printf = xSemaphoreCreateMutex();
  if(g_xMutex_Printf == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // ========== 在这里创建 RTOS定时器事件 ==========
	g_BuzzerSuccessTimer = xTimerCreate(
    "BuzzerOK",                  // 定时器名字
    pdMS_TO_TICKS(10),           // 定时时间：10ms
    pdFALSE,                     // 不自动重复，单次定时器
    NULL,                        // Timer ID
    Timer_SuccessCallback        // 时间到了以后调用的回调
);

/* 失败提示定时器 */
g_BuzzerFailTimer = xTimerCreate(
    "BuzzerERR",                 // 定时器名字
    pdMS_TO_TICKS(10),            // 定时时间：10ms
    pdFALSE,                      // 单次定时器
    NULL,                         // Timer ID
    Timer_FailCallback            // 时间到了以后调用的回调
);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	//STP23L模块队列模块
	g_xQueueSTP23L_Ori = xQueueCreate(3, sizeof(OriData_STP23L_t));
  if (g_xQueueSTP23L_Ori == NULL)//如果内存不足创建失败，返回NULL
  {
    Error_Handler();
  }
	//蓝牙软件队列模块
	g_xQueueBlueTooth_Ori = xQueueCreate(128, sizeof(uint8_t));
  if(g_xQueueBlueTooth_Ori == NULL)
  {
    Error_Handler();
  }
	//PS手柄队列模块
	g_xQueueFlyControl = xQueueCreate(1,sizeof(FlyControlType_t));
	if(g_xQueueFlyControl == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//创建Free RTOS调试任务
// #if ( 1 == userconfig_RTOS_DEBUG )
//	 static uint16_t delaytime = 5000;//打印时间间隔，单位tick
//	 xTaskCreate(RTOSDebugTask,"DebugTask",512, &delaytime ,0,NULL);
// #endif
	
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	if (xTaskCreate(attitude_task, "Attitude", 1024, NULL, 3, NULL) != pdPASS)
	{
	  Error_Handler();
	}
	if (xTaskCreate(control_task, "Control", 512, NULL, 2, NULL) != pdPASS)
	{
	  Error_Handler();
	}
	if (xTaskCreate(power_task, "Power", 512, NULL, 1, NULL) != pdPASS)
	{
	  Error_Handler();
	}
	if (xTaskCreate(STP23L_task, "STP23L", 512, NULL, 2, NULL) != pdPASS)
	{
	  Error_Handler();
	}
	if(xTaskCreate(show_task, "show_task", 512, NULL, 1, NULL) != pdPASS)
	{
		 Error_Handler();
	}
	if(xTaskCreate(Bluetooth_task,"BTAPPTask",256,NULL,1,NULL) !=pdPASS)//xTaskCreate()第一个参数必须是void Bluetooth_task(void *param)这样的;
	{
		 Error_Handler();
	}
	if(xTaskCreate(PS2_task, "PS2", 256, NULL, 2, NULL) != pdPASS)
{
    Error_Handler();
}
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* 初始化 USB Host */
    MX_USB_HOST_Init();

    /* 启动 STP23L 激光测距：UART5 + DMA + 空闲中断 */
    HAL_UARTEx_ReceiveToIdle_DMA(
        &huart5,
        DMABuf_oridata_stp23L.Buf,
        userconfig_STP23L_DMABUF_LEN
    );

    /* USER CODE BEGIN StartDefaultTask */
    for (;;)
    {

        /* 让出 CPU */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//#if ( 1 == userconfig_RTOS_DEBUG )
//void RTOSDebugTask(void *param)
//{
//    uint16_t* delaytime = (uint16_t*)param;
//    static char showbuf[800];

//    while( 1 )
//    {
//        if (xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
//        {
//            vTaskGetRunTimeStats(showbuf);
//            printf("TaskName\tUseTime\tCPU\r\n");
//            printf("%s\r\n", showbuf);
//            xSemaphoreGive(g_xMutex_Printf);
//        }
//        vTaskDelay(*delaytime);

//        if (xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
//        {
//            vTaskList(showbuf);
//            printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
//            printf("%s\r\n", showbuf);
//            xSemaphoreGive(g_xMutex_Printf);
//        }
//        vTaskDelay(*delaytime);

//        if (xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
//        {
//            printf("free heap size : %d bytes\r\n\r\n", xPortGetFreeHeapSize());
//            xSemaphoreGive(g_xMutex_Printf);
//        }
//        vTaskDelay(*delaytime);
//    }
//}
//#endif

/* USER CODE END Application */

