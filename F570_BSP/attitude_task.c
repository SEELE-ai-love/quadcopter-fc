#include "attitude_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "Bsp_imu.h"
static float g_latest_roll=0.0f;
static float g_latest_pitch=0.0f;
static float g_latest_yaw=0.0f;

void Attitude_GetLatest(float *roll,float *pitch,float *yaw)
{
	*roll =g_latest_roll;
	*pitch =g_latest_pitch;
	*yaw =g_latest_yaw;
}

void attitude_task(void*param)
{
	IMU_DATA_T imu={0};
	float roll,pitch,yaw;
	
	TickType_t pretime=xTaskGetTickCount();
	
	while(!ICM20948_Init(&hi2c1))
	{
		printf("ICM20948 init failed,retrying...\r\n");
		HAL_Delay(200);
	}
	
	for(;;)
	{
		float dt=1.0f/(float) ATTITUDE_TASK_FREQ;
		if(ICM20948_Update(&imu,&hi2c1))
		{
			Quaternation_Update(imu.gyro.x,imu.gyro.y,imu.gyro.z,imu.accel.x,imu.accel.y,imu.accel.z,dt,&attitude);
			Quaternion_ToEuler(&attitude,&roll,&pitch,&yaw);
			
			g_latest_roll=roll;
			g_latest_pitch=pitch;
			g_latest_yaw=yaw;
			
			printf("%.2f,%.2f,%.2f\r\n",roll,pitch,yaw);
		}
		else {
			printf("IMU Read failed,skip this cycle\r\n");
		}
		}
	  vTaskDelayUntil(&pretime,pdMS_TO_TICKS(1000.0f/ATTITUDE_TASK_FREQ));
	}
