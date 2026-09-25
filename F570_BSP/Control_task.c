#include "Control_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "attitude_task.h"
#include "Dshot.h"
#include "pid.h"
#include "bsp_ps2.h"

#define CONTROL_RASK_FREQ 200//保持200hz的频率


/*============================
        手柄数据
=============================*/
static GamepadInput_t gamepad;
static pMotorInterface_t d_shot=&UserDshotMotor;//定义一个电机指针类型变量
static uint8_t last_Y=0;//按键Y（增加）
static uint8_t last_B=0;//按键B（终止）
static uint8_t last_X=0;//按键X（减小）
/*============================
        姿态数据
=============================*/
float roll;
float pitch;
float yaw;

float gx;
float gy;
float gz;
/*typedef struct{
    uint8_t Telemetry;//遥测位
    short   throttle;//油门值
} dshotMotorVal_t;
*/

/*============================
        电机数据
=============================*/
//电机启动之前是需要发送最低油门一段时间来解锁电机的
static dshotMotorVal_t m1={0,48};
static dshotMotorVal_t m2={0,48};
static dshotMotorVal_t m3={0,48};
static dshotMotorVal_t m4={0,48};
//定义一个电机启动计数位4秒
static uint32_t unlock_motor=0;
//定义一个飞行器启动PID标志位
static uint8_t startFlag=0;
/* ============================================================
 * 基础油门相关
 * ============================================================ */

/* 目标推力
 * 当前先按照你原始 WheelTec 源码的 145g 进行测试
 */
static float target_thrust = 145.0f;

/* 当前电池电压
 *
 * 这里必须替换成你实际 ADC 读取到的电压变量。
 * 先假设叫 g_robotVOL，与你原始 WheelTec 源码保持一致。
 */
extern float g_robotVOL;

/* 计算出来的基础油门 */
static uint16_t base_throttle = 48;
 /*
 =====================================
   获取PS2手柄
 =====================================
*/static int16_t throttle_value = 48;


/*
    油门限制
*/
#define THROTTLE_MIN 48
#define THROTTLE_MAX 1148

/* ============================================================
 * 基础油门计算
 *
 * 输入：
 *     thrust_g : 目标推力，单位 g
 *     voltage  : 当前电池电压，单位 V
 *
 * 输出：
 *     base_throttle : 基础油门
 *
 * 公式来自 MATLAB 二次拟合结果
 * ============================================================ */
static uint16_t Weight_To_Throttle(float thrust_g, float voltage)
{
    float throttle;

    throttle = 1537.96029618f
             + 8.70113970f * thrust_g
             - 225.50577912f * voltage
             - 0.00880428f * thrust_g * thrust_g
             - 0.23781058f * thrust_g * voltage
             + 9.01035198f * voltage * voltage;

    /* 实验有效油门范围：48 ~ 1148 */
    if (throttle < 48.0f)
    {
        throttle = 48.0f;
    }
    if (throttle > 1148.0f)
    {
        throttle = 1148.0f;
    }
    return (uint16_t)throttle;
}

/*
    油门增量处理(PS2手柄实现)
*/
//static void PS2_Throttle_Control(void)
//{
//    /* Y：单击一次，加 50 */
//    if (UserGamepad.getKeyEvent(PS2KEY_1GREEN) == PS2KEYSTATE_SINGLECLICK)
//    {
//        throttle_value += 50;
//    }

//    /* B：单击一次，减 50 */
//    if (UserGamepad.getKeyEvent(PS2KEY_2RED) == PS2KEYSTATE_SINGLECLICK)
//    {
//        throttle_value -= 50;
//    }

//    /* X：单击一次，急停回 48 */
//    if (UserGamepad.getKeyEvent(PS2KEY_4PINK) == PS2KEYSTATE_SINGLECLICK)
//    {
//        throttle_value = 48;
//    }

//    /* 限幅 */
//    if (throttle_value > THROTTLE_MAX)
//    {
//        throttle_value = THROTTLE_MAX;
//    }

//    if (throttle_value < THROTTLE_MIN)
//    {
//        throttle_value = THROTTLE_MIN;
//    }

//    /* 四个电机使用相同油门 */
//    m1.throttle = throttle_value;
//    m2.throttle = throttle_value;
//    m3.throttle = throttle_value;
//    m4.throttle = throttle_value;
//}

void control_task(void*param)
{
	TickType_t pretime;
	(void)param;
	d_shot->init();//d_shot指向init
	pretime=xTaskGetTickCount();
	
	for(;;)
	{
		//获取姿态角
		Attitude_GetLatest(&roll,&pitch,&yaw);
		//获取角速度
	  Attitude_GetGyro(&gx,&gy,&gz);
	    //光流计预留点
		//雷达激光预留点
		if(unlock_motor<CONTROL_RASK_FREQ*4)
{
		unlock_motor++;
		m1.throttle=48;
    m2.throttle=48;
    m3.throttle=48;
    m4.throttle=48;
}//延时四秒，即
		else
		{
		//飞行器启动
		if(startFlag ==1)
		{
			 Attitude_PID_Update(0.0f, 0.0f, 0.0f); //目标姿态角
			base_throttle=Weight_To_Throttle(target_thrust,g_robotVOL);
//PID_update(&RollPID, 0,roll);//外环Roll
//PID_update(&PitchPID, 0,pitch);//外环Pitch
//PID_update(&YawPID, 0,yaw);//外环Yaw
			/*不需要再单独写一遍外环的PID公式，因为已经在Attitude_PID_Update()里面写过了:PID_update(&RollPID,roll_target,roll); PID_update(&PitchPID,pitch_target,pitch);PID_update(&YawPID,yaw_target,yaw);*/
			PID_update(&RollRatePID, RollRateTarget, gx);//内环RollRate
      PID_update(&PitchRatePID, PitchRateTarget, gy);//内环PitchRate
      PID_update(&YawRatePID, YawRateTarget, gz);//内环YawRate
			
			m1.throttle =base_throttle+RollRatePID.output + PitchRatePID.output + YawRatePID.output;
			m2.throttle =base_throttle+RollRatePID.output - PitchRatePID.output - YawRatePID.output;
			m3.throttle =base_throttle-RollRatePID.output - PitchRatePID.output + YawRatePID.output;
			m4.throttle =base_throttle-RollRatePID.output + PitchRatePID.output - YawRatePID.output;
	
if (m1.throttle < THROTTLE_MIN)
    m1.throttle = THROTTLE_MIN;
if (m1.throttle > THROTTLE_MAX)
    m1.throttle = THROTTLE_MAX;

if (m2.throttle < THROTTLE_MIN)
    m2.throttle = THROTTLE_MIN;
if (m2.throttle > THROTTLE_MAX)
    m2.throttle = THROTTLE_MAX;

if (m3.throttle < THROTTLE_MIN)
    m3.throttle = THROTTLE_MIN;
if (m3.throttle > THROTTLE_MAX)
    m3.throttle = THROTTLE_MAX;

if (m4.throttle < THROTTLE_MIN)
    m4.throttle = THROTTLE_MIN;
if (m4.throttle > THROTTLE_MAX)
    m4.throttle = THROTTLE_MAX;

		}
		    else
    {
        /* 没有正式启动控制时，保持停止 */
        PID_Reset(&RollPID);
        PID_Reset(&PitchPID);
        PID_Reset(&YawPID);
        PID_Reset(&RollRatePID);
        PID_Reset(&PitchRatePID);
        PID_Reset(&YawRatePID);

        m1.throttle = 48;
        m2.throttle = 48;
        m3.throttle = 48;
        m4.throttle = 48;
    }
//PS2_Throttle_Control();手柄控制检测电机
			
			//电机转动
//	    m1.throttle=120;
//		  m2.throttle=120;
//			m3.throttle=120;
//			m4.throttle=120;
		}
		d_shot->set_target(m1,m2,m3,m4);//d_shot指向set_target
		vTaskDelayUntil(&pretime,pdMS_TO_TICKS(1000.0f/CONTROL_RASK_FREQ));
	}
}
