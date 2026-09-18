#include "power_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "bsp_adc.h"
#include "semphr.h"

//========== 互斥锁 ==========//
extern SemaphoreHandle_t g_xMutex_Printf; 

/* ============================================================
 * ADC读出来的是0~4095的原始数值（12位ADC），不是电压本身。
 * 要换算成实际电压，公式是：
 *
 *   实际电压 = ADC原始值 / 4095 * 参考电压(通常3.3V) * 分压比
 *
 * "分压比"取决于F570板子上电压采样电路具体用了多大的分压电阻——
 * 电阻分压把电压降到ADC能测的0~3.3V范围内，再送进PA7。
 * ============================================================ */
#define ADC_REF_VOLTAGE      3.3f   //ADC采集引脚的电压范围：0~3.3V
#define ADC_RESOLUTION       4095.0f //ADC的精度，2的12次方
#define VOLTAGE_DIVIDER_RATIO  11.0f //分压比11：1 [(1/（VIN下臂电阻/总电阻）]
#define CURRENT_SENSE_RATIO      5.0f//电流比 （1/电阻*增益系数） 

/* 低电压保护阈值*/
#define LOW_VOLTAGE_THRESHOLD  10.5f  /*电压的阈值，低于10.5f表示电源电压不够*/

static float g_latest_voltage = 0.0f;//存放电压
static float g_latest_current = 0.0f;//存放电流

void power_task(void *param)
{
    TickType_t preTime;//preTime是RTOS里面负责记录时间戳的变量
    (void)param;
    /* ADC+DMA只需要启动一次，之后硬件自己持续采样 */
    UserADC1.init();
    preTime = xTaskGetTickCount();//获取当前的Tick数

    for (;;)
    {
        uint16_t raw_vbat = UserADC1.getValue(userconfigADC_VBAT_CHANNEL);//raw_vbat表示电压
        uint16_t raw_curr = UserADC1.getValue(userconfigADC_Curr_CHANNEL);//raw_curr表示电流
			 /* ============================================================
       * 电源电压的测量：电压=（ADC/2^12)*3.3*11.0f;
       * ============================================================ */
        float voltage = (raw_vbat / ADC_RESOLUTION) * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
			 /* ============================================================
       * 电源电流的测量：电流=(ADC/2^12)*3.3*5.0f
       * ============================================================ */
        float current = (raw_curr / ADC_RESOLUTION) * ADC_REF_VOLTAGE * CURRENT_SENSE_RATIO;
	      /* 进入临界区，关闭中断 */
        taskENTER_CRITICAL();
			
        g_latest_voltage = voltage;
        g_latest_current = current;
			
				/* 退出临界区，开启中断 */
        taskEXIT_CRITICAL();
        /* ============================================================
         * 【安全预留点——低电压保护】
         * ============================================================ */
				g_robotVOL = voltage;
        if (voltage < LOW_VOLTAGE_THRESHOLD && voltage > 1.0f) {
            /* voltage > 1.0f 是为了排除"分压系数还没配对、读出接近0
             * 明显异常值，避免上电瞬间还没接电池时就疯狂报警 */
            //printf("WARNING: low battery! %.2fV\r\n", voltage);//打印报警
        }
					if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
        {
      //printf("VBAT: %.2fV, CURR: %.2fA\r\n", voltage, current);//打印电压与电流
		   xSemaphoreGive(g_xMutex_Printf); 
        vTaskDelayUntil(&preTime, pdMS_TO_TICKS(1000.0f / POWER_TASK_FREQ));
        }
}
		}

void Power_GetLatest(float *voltage, float *current)//跨任务接口函数
{
    taskENTER_CRITICAL();
    *voltage = g_latest_voltage;
    *current = g_latest_current;
    taskEXIT_CRITICAL();
}
