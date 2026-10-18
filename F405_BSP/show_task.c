#include "show_task.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
#include "attitude_task.h"
#include "pid.h"

void show_task(void *param)
{
    float roll;
    float pitch;
    float yaw;

    float gx;
    float gy;
    float gz;

    char showbuffer[256];

    TickType_t pretime;
    (void)param;
    pretime = xTaskGetTickCount();

    while (1)
    {
        /* 获取姿态角 */
        Attitude_GetLatest(&roll, &pitch, &yaw);
        /* 直接调用已经写好的角速度接口 */
        Attitude_GetGyro(&gx, &gy, &gz);

        /*
         * WHEELTEC：
         *
         * CH0 = Roll
         * CH1 = Pitch
         * CH2 = Yaw
         *
         * CH3 = Gx
         * CH4 = Gy
         * CH5 = Gz
         *
         * CH6 = RollRateTarget
         * CH7 = PitchRateTarget
         * CH8 = YawRateTarget
         */
//			 sprintf(showbuffer,
//                "{B1.000:2.000:3.000:4.000:5.000:6.000:7.000:8.000:9.000}$");
//        sprintf(showbuffer,
//                "{B%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f}$",
//                roll,
//                pitch,
//                yaw,
//                gx,
//                gy,
//                gz,
//                RollRateTarget,
//                PitchRateTarget,
//                YawRateTarget);
        HAL_UART_Transmit(&huart4,
                          (uint8_t *)showbuffer,
                          strlen(showbuffer),
                          100);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
