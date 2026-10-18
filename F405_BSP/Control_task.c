#include "Control_task.h" 
#include "FreeRTOS.h" 
#include "task.h" 
#include <math.h> 
#include "attitude_task.h" 
#include "Dshot.h" 
#include "pid.h" 
#include "Bsp_ps2.h" 
#include "power_task.h" 
#include <stdio.h> 
#include "show_task.h" 
#include "queue.h" 
#include "PS2_task.h" 
#include <stdio.h> 
#include "Bsp_stp23L.h"
#include "semphr.h"
#include "Bsp_buzzer.h"
#include <string.h>
#include "Position_hold.h"
 
#define CONTROL_RASK_FREQ 200//保持200hz的频率 
/* Mixer mode: 0 = original mixer path, 1 = user attitude-axis mixer */
#define FLIGHT_MIXER_MODE 1
#define STARTUP_GYRO_THRESHOLD 0.02f//gyro的阈值
#define HEIGHT_SPEED_MAX 0.5f// 最大升降速度 0.5m/s

// PD_CONTROL BEGIN：桌面只计算位置修正和油门，不能给电机发信号
#define POS_DESKTOP_TEST   1U
#define POS_DESKTOP_BASE   770U//用于和之前日志对照，不代表实际悬停油门
// PD_CONTROL END
/*============================ 
        手柄数据 
=============================*/ 
//static GamepadInput_t gamepad; 
// PD_CONTROL：正式模式才编译这项，桌面保留原代码
#if POS_DESKTOP_TEST == 0U
static pMotorInterface_t d_shot=&UserDshotMotor;//定义一个电机指针类型变量 
#endif
//static uint8_t last_Y=0;//按键Y（增加） 
//static uint8_t last_B=0;//按键B（终止） 
//static uint8_t last_X=0;//按键X（减小） 
static float roll_control = 0.0f;//手柄映射Roll 
static float pitch_control = 0.0f;//手柄映射Pitch 
static float yaw_control=0.0f;//手柄映射Yaw 

/* 当前最高控制优先级 */ 
static uint8_t controlCmdNumber = IDLECmd; 
 
/* 连续丢失控制指令计数 */ 
//static uint16_t LostCmdCnt = 0; 
 
/* 当前最新的PS2控制指令 */ 
static FlyControlType_t controlVal = {0}; 
 
/* 连续没有收到新指令的次数 */ 
static uint16_t refresh_cmdstate = 0;
/*============================ 
        互斥锁 
=============================*/ 
 extern SemaphoreHandle_t g_xMutex_Printf; 
/*============================ 
        姿态数据 
=============================*/ 
float roll; 
float pitch; 
float yaw; 
 
float gx; 
float gy; 
float gz; 
 
 
//static uint8_t yaw_print_cnt = 0; 
/*typedef struct{ 
    uint8_t Telemetry;//遥测位 
    short   throttle;//油门值 
} dshotMotorVal_t; 
*/ 
static uint16_t print_cnt = 0; 
static float yawcontrol=0;//yaw修正
/*============================ 
        高度数据 
=============================*/ 
static float height_target = 0.0f;//高度环目标值(离地高度）
static float height_current = 0.0f;//高度实际值（离地高度）
static float height_speed = 0.0f;//高度速度值
static float zero_distance = 0.0f;//高度零点
float use_distance = 0.0f;//实际距离地面高度（离地高度）
static float last_height = 0.0f;       // 上一次的高度，单位 m
static float last_height_speed = 0.0f; // 上一次高度速度，单位 m/s
static float prev_distance = 0.0f;//上一次的离地高度
static uint8_t first_frame = 1;//是否第一次拿到激光数据：1 = 是，只存基准，不做差分
/* ============================================================ 
         零点数据 
 * ============================================================ */ 
 
static uint8_t startup_calib_done = 0;      // 标志位，0=未完成，1=完成
static uint32_t startup_count = 0;           // 静止采样次数

static float yaw_sum = 0.0f;                 // Yaw累计
static float distance_sum = 0.0f;            // STP23L距离累计
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
// PD_CONTROL BEGIN：长按START请求重新捕获航向，只在开始控制时处理一次
static uint8_t yaw_capture_pending = 0U;
// PD_CONTROL END 
/* ============================================================ 
 * 电源数据 /电压/电流
 * ============================================================ */ 
static float battery_voltage = 0.0f; 
static float battery_current = 0.0f; 

/* ============================================================ 
 * 基础油门相关 /目标推力  140g/基础油门 
 * ============================================================ */ 
// PD_CONTROL：正式模式才编译这项，桌面保留原代码
#if POS_DESKTOP_TEST == 0U
static float target_thrust = 140.0f; 
#endif
static uint16_t base_throttle = 48; 
 /* 
 ===================================== 
 获取PS2手柄，手柄调试下给的最小油门 
 ===================================== 
*/ //static int16_t throttle_value = 48; 
 
/* 
    油门与最低电压限制 
*/ 
#define THROTTLE_MIN 48 
#define THROTTLE_MAX 1500 
#define START_VOLTAGE_MIN 10.8f
 
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
// PD_CONTROL：桌面使用固定对照油门，推力拟合只在正式模式编译
#if POS_DESKTOP_TEST == 0U
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
//		printf("V=%.2f Base=%u\r\n", 
//       battery_voltage, 
//       base_throttle); 
    return (uint16_t)throttle; 
		//打印检测是否是基础油门不够导致无人机无法离地飞行 
	} 
#endif

/* 
    油门增量处理PS2手柄实现（已实现）
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

static void PS2_startFlag_Control(void) 
{ 
    PS2KEY_State_t start_event; 

    start_event = UserGamepad.getKeyEvent(PS2KEY_START); 

    switch(start_event) 
    { 
    case PS2KEYSTATE_LONGCLICK:
    // PD_CONTROL：桌面只算命令，不需要电池；正式模式仍保留原来的电压保护
    if (POS_DESKTOP_TEST != 0U || battery_voltage >= START_VOLTAGE_MIN)//电压保护
    {
        startFlag = 1;
        // PD_CONTROL：重复长按也重新捕获，不能只依赖第一次进入
        yaw_capture_pending = 1U;

        PID_Reset(&RollPID);
        PID_Reset(&PitchPID);
        PID_Reset(&YawPID);
        PID_Reset(&RollRatePID);
        PID_Reset(&PitchRatePID);
        PID_Reset(&YawRatePID);
    }
    else
    {
        startFlag = 0;
    }
    break;

        case PS2KEYSTATE_SINGLECLICK: 
            startFlag = 0; 
            break; 

        case PS2KEYSTATE_DOUBLECLICK: 
            startFlag = 0; 
            break; 

        default: 
            break; 
    } 
    // PD_CONTROL BEGIN：按键只提示启停状态，不额外占一个周期打印
#if POS_DESKTOP_TEST == 1U
    if(start_event != PS2KEYSTATE_NONE && xSemaphoreTake(g_xMutex_Printf,0U) == pdTRUE)
    {
        printf("POS_KEY run=%u cal=%u TX=0\r\n",(unsigned int)startFlag,(unsigned int)startup_calib_done);
        xSemaphoreGive(g_xMutex_Printf);
    }
#endif
    // PD_CONTROL END
}
 
////手柄映射：控制量输入（这里转换，考虑使用RTOS队列的形式） 
//static void PS2_Attitude_Control(void) 
//{ 
//	//关于手柄映射： 
//		float yaw_step; 
//    yaw_step = 90.0f / (float)CONTROL_RASK_FREQ;//Yaw步进值，这里是每次转动旋钮是让飞行器偏航90度，由于control_task任务是200hz，这里计算平均该走多少度 
//    if(gamepad.LY < 64) 
//    { 
//        /* 手柄向前 → Roll负 */ 
//        roll_control = -10.0f; 
//    } 
//    else if(gamepad.LY > 190) 
//    { 
//        /* 手柄向后 → Roll正 */ 
//        roll_control =10.0f; 
//    } 
//    else 
//    { 
//        /* 摇杆中间死区 */ 
//        roll_control = 0.0f; 
//    } 
//		    if(gamepad.LX < 64) 
//    { 
//        /* 手柄向左 → Pitch负 */ 
//        pitch_control = -10.0f; 
//    } 
//    else if(gamepad.LX > 190) 
//    { 
//        /* 手柄向右 → Pitch正 */ 
//        pitch_control =10.0f; 
//    } 
//    else 
//    { 
//        /* 摇杆中间死区 */ 
//        pitch_control = 0.0f; 
//    } 
//    if(gamepad.RX < 64) 
//    { 
//        /* 右摇杆向左为正 */ 
//        yaw_control += yaw_step; 
//    } 
//    else if(gamepad.RX > 190) 
//    { 
//        /* 右摇杆向右为负 */ 
//        yaw_control -= yaw_step; 
//    } 
//		else  
//		{ 
//		} 
//} 
static const float yaw_step_rad = (90.0f * 0.017453292519943f) / (float)CONTROL_RASK_FREQ;//yaw手柄步进值
/*============================
        光流位置环
=============================*/
//方向定义已经明确，但是闭环输出方向还需要拆桨验证，所以先保持0
#define POS_DIRECTION_CONFIRMED 0U
//光流现在仍按0.50m换算，实际离地尺度确认之后再打开
#define POS_SCALE_CONFIRMED     0U


//以下参数是工程窗口，需要人类结合串口数据和拆桨输出验证
#define POS_RECOVERY_FRAMES     10U
#define POS_RECOVERY_MS         200U
#define POS_SAMPLE_GAP_MS       60U
#define POS_CONTROL_GAP_MS      25U
#define POS_SPEED_LIMIT         1.0f
#define POS_LOCK_SPEED_LIMIT    0.30f
#define POS_ERROR_LIMIT         0.80f
#define POS_JUMP_MARGIN         0.03f
#define POS_TILT_LIMIT          0.34906585f
#define POS_YAW_LIMIT           0.17453293f
#define POS_SLEW_RAD_S          0.35f

typedef enum
{
    CONTROL_POS_OFF = 0,
    CONTROL_POS_WAIT,
    CONTROL_POS_ACTIVE,
    CONTROL_POS_FAULT
} ControlPosMode_t;

typedef struct
{
    ControlPosMode_t mode;
    uint8_t have_call;
    uint8_t have_sequence;
    uint8_t have_sample;
    uint32_t last_call_ms;
    uint32_t sequence;
    uint32_t fault_epoch;
    uint32_t rx_epoch;
    uint32_t sample_ms;
    uint32_t recovery_start_ms;
    uint16_t good_frames;
    float last_x;
    float last_y;
    float target_x;
    float target_y;
    float yaw_reference;
    float roll_add;
    float pitch_add;
} ControlPosState_t;

static ControlPosState_t control_pos = {0};
static uint8_t position_hold_enable = 1U;//定点功能开关，先保持关闭
// PD_CONTROL：当前已启用位置计算，上行旧注释保留；电机仍由桌面模式禁用

//老师的0.5/0.7只是待调起点，位置环输出直接是rad，不再乘角度转换系数
static const PositionHoldConfig_t position_config =
{
    .kp_x = 0.5f,              //rad/m，待调
    .kp_y = 0.5f,              //rad/m，待调
    .kv_x = 0.7f,              //rad/(m/s)，待调
    .kv_y = 0.7f,              //rad/(m/s)，待调
    .angle_limit = 0.35f //约5度，待验证
    //上行5度是旧demo说明，现在按老师的0.35rad限幅，约20度；启用标志仍为0
};

#if POS_DESKTOP_TEST == 0U
// PD_CONTROL BEGIN：开环验证，只算和打印，不写control_pos或姿态目标
static volatile uint8_t pd_control_relock = 1U;//需要重新选测试原点时置1，默认首次自动记录
static void Control_PD_OpenLoop(void)
{
    static TickType_t last_print_tick = 0U;
    static uint8_t target_locked = 0U;
    static float target_x = 0.0f,target_y = 0.0f;
    static uint32_t fault_epoch = 0U,rx_epoch = 0U;
    FlyPosMsg_t estimate;
    LC307_Status_t status;
    PositionHoldOutput_t output;
    TickType_t now = xTaskGetTickCount();
    uint8_t valid;

    if((TickType_t)(now-last_print_tick) < pdMS_TO_TICKS(500U)) return;
    last_print_tick = now;
    if(startFlag != 0U || startup_calib_done == 0U) return;
    valid = LC307_GetLatestEx(&estimate,&status);
    if(valid == 0U)
    {
        target_locked = 0U;
        if(xSemaphoreTake(g_xMutex_Printf,0U) == pdTRUE)
        {
            printf("PD valid=0 age_ms=%lu\r\n",(unsigned long)status.age_ms);
            xSemaphoreGive(g_xMutex_Printf);
        }
        return;
    }
    if(target_locked == 0U || pd_control_relock != 0U ||
       fault_epoch != status.fault_epoch || rx_epoch != status.rx_epoch)
    {
        //先停稳再记录测试原点；失效恢复后重新记录，不追旧目标
        if(fabsf(estimate.x_speed) > POS_LOCK_SPEED_LIMIT ||
           fabsf(estimate.y_speed) > POS_LOCK_SPEED_LIMIT) return;
        target_x = estimate.x_pos;
        target_y = estimate.y_pos;
        fault_epoch = status.fault_epoch;
        rx_epoch = status.rx_epoch;
        target_locked = 1U;
        pd_control_relock = 0U;
    }
    if(PositionHold_Compute(&estimate,target_x,target_y,&position_config,&output) == 0U)
    {
        target_locked = 0U;
        return;
    }
    //打印出来PD误差，实际速度，PD输出值（姿态角度）验证PD计算的正确性
    if(xSemaphoreTake(g_xMutex_Printf,0U) == pdTRUE)
    {
        // 误差和速度用mm、mm/s，u和命令用mrad；所有输出只供观察
        // 串口打印：在已确认光流预测速度和光流预测位移的前提下，证明向着正方向运动最后
        // printf("PD valid=1 target_mm=%ld,%ld error_mm=%ld,%ld v_mm_s=%ld,%ld "
        //        "u_mrad=%ld,%ld roll_cmd=%ld pitch_cmd=%ld\r\n",
        //        (long)(target_x*1000.0f),(long)(target_y*1000.0f),
        //        (long)((target_x-estimate.x_pos)*1000.0f),
        //        (long)((target_y-estimate.y_pos)*1000.0f),
        //        (long)(estimate.x_speed*1000.0f),(long)(estimate.y_speed*1000.0f),
        //        (long)(output.control_x*1000.0f),(long)(output.control_y*1000.0f),
        //        (long)(output.roll_add*1000.0f),(long)(output.pitch_add*1000.0f));
        xSemaphoreGive(g_xMutex_Printf);
    }
}
// PD_CONTROL END
#endif

static float Control_PosLimit(float value,float limit)
{
    if(value > limit) return limit;
    if(value < -limit) return -limit;
    return value;
}

static float Control_PosYawError(float a,float b)
{
    float error = a-b;
    while(error > 3.14159265f) error -= 6.28318531f;
    while(error < -3.14159265f) error += 6.28318531f;
    return error;
}

static void Control_PosReset(void)
{
    memset(&control_pos,0,sizeof(control_pos));
}

static void Control_PosDrop(ControlPosMode_t mode,const LC307_Status_t *status)
{
    //这里只复位位置控制，不清光流位置和速度，也不复位姿态内环
    control_pos.mode = mode;
    control_pos.have_sample = 0U;
    control_pos.good_frames = 0U;
    control_pos.target_x = 0.0f;
    control_pos.target_y = 0.0f;
    control_pos.roll_add = 0.0f;
    control_pos.pitch_add = 0.0f;

    control_pos.sequence = status->sequence;
    control_pos.fault_epoch = status->fault_epoch;
    control_pos.rx_epoch = status->rx_epoch;
    control_pos.sample_ms = status->receive_ms;
    control_pos.have_sequence = 1U;
}

static void Control_PositionHold(float *roll_target,float *pitch_target)
{
    FlyPosMsg_t estimate;
    LC307_Status_t status;
    PositionHoldOutput_t correction;
    uint8_t valid;
    uint32_t now_ms;
    uint32_t control_dt_ms = 5U;
    uint32_t sample_dt_ms;
    float jump_limit;
    float max_step;

    *roll_target = isfinite(roll_control) ? roll_control : 0.0f;
    *pitch_target = isfinite(pitch_control) ? pitch_control : 0.0f;

    valid = LC307_GetLatestEx(&estimate,&status);
    //当前Tick频率是1000Hz，1个Tick就是1ms
    now_ms = (uint32_t)xTaskGetTickCount();

    if(control_pos.have_call != 0U)
    {
        control_dt_ms = (uint32_t)(now_ms-control_pos.last_call_ms);
    }
    control_pos.last_call_ms = now_ms;
    control_pos.have_call = 1U;

    //关闭定点或停止飞行，才解除位置超差的锁存故障
    if(position_hold_enable == 0U || startFlag == 0U || startup_calib_done == 0U)
    {
        Control_PosReset();
        return;
    }
    if(control_pos.mode == CONTROL_POS_FAULT)
    {
        return;
    }

    // PD_CONTROL BEGIN：正式飞行门限保留，桌面5mm只验证计算，不冒充尺度已通过
    if(POS_DESKTOP_TEST == 0U &&
       (POS_DIRECTION_CONFIRMED == 0U || POS_SCALE_CONFIRMED == 0U ||
        !isfinite(height_current) || fabsf(height_current-0.50f) >= 0.08f))
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }
    // PD_CONTROL END
    if(refresh_cmdstate >= CONTROL_RASK_FREQ/4U || controlVal.source == IDLECmd ||
       valid == 0U || status.age_ms > 100U || control_dt_ms > POS_CONTROL_GAP_MS)
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }

    if(!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) ||
       !isfinite(roll_control) || !isfinite(pitch_control) ||
       !isfinite(yaw_control) || !isfinite(controlVal.height) ||
       !isfinite(estimate.x_pos) || !isfinite(estimate.y_pos) ||
       !isfinite(estimate.x_speed) || !isfinite(estimate.y_speed))
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }

    //手柄介入、转向或者速度和倾角过大，先退出定点，交回原姿态控制
    if(fabsf(roll_control) > 0.01f || fabsf(pitch_control) > 0.01f ||
       fabsf(yaw_control) > 0.5f || fabsf(controlVal.height) > 0.0001f ||
       fabsf(roll) > POS_TILT_LIMIT || fabsf(pitch) > POS_TILT_LIMIT ||
       fabsf(estimate.x_speed) > POS_SPEED_LIMIT ||
       fabsf(estimate.y_speed) > POS_SPEED_LIMIT)
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }

    if(control_pos.have_sequence != 0U &&
       (status.fault_epoch != control_pos.fault_epoch ||
        status.rx_epoch != control_pos.rx_epoch))
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }
    control_pos.fault_epoch = status.fault_epoch;
    control_pos.rx_epoch = status.rx_epoch;

    if(control_pos.have_sequence == 0U || status.sequence != control_pos.sequence)
    {
        //同一DMA块可以有多帧，序号变化但接收时间相同是正常情况
        if(control_pos.have_sequence == 0U || status.receive_ms != control_pos.sample_ms)
        {
            if(control_pos.have_sample != 0U)
            {
                sample_dt_ms = (uint32_t)(status.receive_ms-control_pos.sample_ms);
                if(sample_dt_ms > POS_SAMPLE_GAP_MS)
                {
                    Control_PosDrop(CONTROL_POS_WAIT,&status);
                    return;
                }

                //只有不同接收时间组才检查跳变，不拿零时间间隔计算
                jump_limit = POS_SPEED_LIMIT*((float)sample_dt_ms*0.001f)+POS_JUMP_MARGIN;
                if(fabsf(estimate.x_pos-control_pos.last_x) > jump_limit ||
                   fabsf(estimate.y_pos-control_pos.last_y) > jump_limit)
                {
                    Control_PosDrop(CONTROL_POS_WAIT,&status);
                    return;
                }
            }
            else
            {
                control_pos.recovery_start_ms = status.receive_ms;
                control_pos.yaw_reference = yaw;
            }

            control_pos.sample_ms = status.receive_ms;
            control_pos.have_sample = 1U;

            if(control_pos.mode != CONTROL_POS_ACTIVE)
            {
                if(fabsf(estimate.x_speed) > POS_LOCK_SPEED_LIMIT ||
                   fabsf(estimate.y_speed) > POS_LOCK_SPEED_LIMIT)
                {
                    Control_PosDrop(CONTROL_POS_WAIT,&status);
                    return;
                }

                //一个接收时间组最多累计一次，不按序号差值补算恢复计数
                if(control_pos.good_frames < POS_RECOVERY_FRAMES)
                {
                    control_pos.good_frames++;
                }
                if(control_pos.good_frames >= POS_RECOVERY_FRAMES &&
                   (uint32_t)(status.receive_ms-control_pos.recovery_start_ms) >= POS_RECOVERY_MS)
                {
                    //松杆或恢复之后，重新锁当前位置，不追赶失效前的旧目标
                    control_pos.target_x = estimate.x_pos;
                    control_pos.target_y = estimate.y_pos;
                    control_pos.mode = CONTROL_POS_ACTIVE;
                }
                else
                {
                    control_pos.mode = CONTROL_POS_WAIT;
                }
            }
        }

        //同时间组的新帧也更新最新位置基准，但是不重复累计恢复计数
        control_pos.sequence = status.sequence;
        control_pos.have_sequence = 1U;
        control_pos.last_x = estimate.x_pos;
        control_pos.last_y = estimate.y_pos;
    }

    if(control_pos.mode != CONTROL_POS_ACTIVE)
    {
        return;
    }
    if(fabsf(Control_PosYawError(yaw,control_pos.yaw_reference)) > POS_YAW_LIMIT)
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }

    if(fabsf(control_pos.target_x-estimate.x_pos) > POS_ERROR_LIMIT ||
       fabsf(control_pos.target_y-estimate.y_pos) > POS_ERROR_LIMIT)
    {
        Control_PosDrop(CONTROL_POS_FAULT,&status);
        return;
    }

    if(PositionHold_Compute(&estimate,control_pos.target_x,control_pos.target_y,
                            &position_config,&correction) == 0U)
    {
        Control_PosDrop(CONTROL_POS_WAIT,&status);
        return;
    }

    max_step = POS_SLEW_RAD_S*((float)control_dt_ms*0.001f);
    control_pos.roll_add += Control_PosLimit(correction.roll_add-control_pos.roll_add,max_step);
    control_pos.pitch_add += Control_PosLimit(correction.pitch_add-control_pos.pitch_add,max_step);

    //位置修正已经按当前取负后的姿态定义计算，这里直接相加，不再取负
    *roll_target += control_pos.roll_add;
    *pitch_target += control_pos.pitch_add;
}

//飞控核心：控制层 
void control_task(void*param) 
{ 
	TickType_t pretime; 
	(void)param; 
// PD_CONTROL BEGIN：桌面模式禁止电机初始化和发送
#if POS_DESKTOP_TEST == 0U
	//d_shot->init();//d_shot指向init 
#endif
// PD_CONTROL END
	pretime=xTaskGetTickCount(); 
	 
	//由于飞控板没有磁力计，所以yaw位置不定，导致无人机会产生偏转，所以这里不规定目标偏航角是0度，而是实际测量角度 
	static uint8_t startevent=1; 
for(;;) 
	{ 
        float roll_target;
        float pitch_target;
		//获取姿态角 
		Attitude_GetLatest(&roll,&pitch,&yaw); 
		//获取角速度 
	  Attitude_GetGyro(&gx,&gy,&gz); 
        //200Hz预测，只有新光流帧才校正，位置模块仍由Control唯一调用
        LC307_EstimatorUpdate();
        // PD_CONTROL BEGIN：未启动也能验证，结果不接入姿态PID
        // PD_CONTROL BEGIN：桌面由现有位置控制器计算，旧开环打印先不重复运行
#if POS_DESKTOP_TEST == 0U
        Control_PD_OpenLoop();
#endif
        // PD_CONTROL END
        // PD_CONTROL END
 
    //获取电源数据 
    Power_GetLatest(&battery_voltage, &battery_current); 
	  //光流计预留点

		//手柄开启飞行任务 
        // PD_CONTROL BEGIN：长按START开始计算，单击或双击停止；桌面不发电机信号
		PS2_startFlag_Control();  
        // PD_CONTROL END
/* ============================================================ 
  *零点标定阶段
	*由于飞控板没有磁力计，所以yaw位置不定，导致无人机会产生偏转，所以这里不规定目标偏航角是0度，而是实际测量角度
 * ============================================================ */ 
		if(startup_calib_done == 0)				//零点标定，平均5秒取平均值
{
    /* 判断飞机当前是否基本静止，每个gyro取绝对值之后都要小于阈值然后最低高度必须大意给定的阈值 */
    if(fabsf(gx) < STARTUP_GYRO_THRESHOLD &&
       fabsf(gy) < STARTUP_GYRO_THRESHOLD &&
       fabsf(gz) < STARTUP_GYRO_THRESHOLD)
    {
        /* 累计Yaw */
        yaw_sum += yaw;

        /* 累计STP23L距离 */
        distance_sum += g_readonly_distance;

        /* 采样次数+1 */
        startup_count++;

        /* 连续静止5秒 */
        if(startup_count >= CONTROL_RASK_FREQ * 5)
        {
            /* 取平均值,平均5秒 */
            yawcontrol = yaw_sum / (float)startup_count;

            zero_distance = distance_sum / (float)startup_count;

            /* 标记：启动标定完成 */
            startup_calib_done = 1;
					  /* 激动蜂鸣器*/
	          xTimerStart(g_BuzzerSuccessTimer, 0);
            /* 当前相对高度定义为0 */
            use_distance = 0.0f;
            height_current = 0.0f;
					  height_speed =0.0f;
					
						/* 速度计算历史值清零 */
			      last_height =0.0f;
			      last_height_speed =0.0f;
        }
    }
    else
    {
        /* 中途动了，重新开始积累，5秒计时 */
        startup_count = 0;
        yaw_sum = 0.0f;
        distance_sum = 0.0f;

    }
}
/* ============================================================ 
  *摇杆队列阶段 
* ============================================================ */ 
if (xQueueReceive(g_xQueueFlyControl, &controlVal, 0) == pdPASS)//接收队列成功 
{ 
    refresh_cmdstate = 0; 
 
    roll_control  = controlVal.roll; 
    pitch_control = controlVal.pitch; 
    yaw_control   = controlVal.gyroz; 
	
	  height_target += controlVal.height;
} 
  else 
        { 
            /* 
             * 当前5ms控制周期没有收到新数据。 
             * 
             * PS2只有20Hz，所以这属于正常情况， 
             * 不能因为一次没收到就清零。 
             */ 
            refresh_cmdstate++; 
            /* 
             * 连续250ms没有收到控制指令， 
             * 认为控制输入已经失效。 
             */ 
            if(refresh_cmdstate >= CONTROL_RASK_FREQ / 4) 
            { 
                roll_control  = 0.0f; 
                pitch_control = 0.0f; 
                yaw_control   = 0.0f; 
                refresh_cmdstate = CONTROL_RASK_FREQ / 4 + 1; 
 
                /* 控制来源回到IDLE */ 
                controlCmdNumber = IDLECmd; 
            } 
        } 
        /* ============================================================ 
         * 3. Yaw目标角更新 
         * PS2的gyroz： 
         *     +1：向一个方向旋转 
         *     -1：向另一个方向旋转 
         *      0：停止旋转 
         * Control_task是200Hz， 
         * 每个周期改变0.45° 
         * 200 × 0.45 = 90°/s 
         * ============================================================ */ 
//判断yaw是不是正方向 
if(yaw_control > 0.5f)
{ 
    yawcontrol += yaw_step_rad; 
} 
else if(yaw_control < -0.5f) 
{ 
    yawcontrol -= yaw_step_rad; 
} 
// Yaw目标角归一化到 [-180°, 180°] 
if (yawcontrol > 3.14159265f) 
{ 
    yawcontrol -= 6.28318531f; 
} 
else if (yawcontrol < -3.14159265f) 
{ 
    yawcontrol += 6.28318531f; 
} 
//高度限幅
if(height_target > 1.5f)
{
    height_target = 1.5f;
}

if(height_target < 0.5f)
{
    height_target = 0.5f;
}
 
 /* ============================================================ 
         *串口打印日志1，用来验证姿态角和yaw修正是否正确
 * ============================================================ */
//print_cnt++;

//if(print_cnt >= CONTROL_RASK_FREQ / 10)
//{
//    print_cnt = 0;

//    if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
//    {
//        printf("ATT | CAL=%d | R=%.3f P=%.3f Y=%.3f | YT=%.3f\r\n",
//               startup_calib_done,
//               roll *57.2957795f,
//               pitch *57.2957795f,
//               yaw *57.2957795f,
//               yawcontrol *57.2957795f);

//        xSemaphoreGive(g_xMutex_Printf);
//    }
//}
 /* ============================================================ 
         *电机解锁阶段 
 * ============================================================ */ 
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
		//matlab拟合曲线之后打印出飞行器的基础油门 
	  //printf("V=%.2fV, BaseThrottle=%u\r\n", battery_voltage,Weight_To_Throttle(140.0f, 10.0f)); 
		//飞行器启动*4秒电机+零点标记完成*5秒标记
		if(startFlag == 1 && startup_calib_done == 1) 
		{ 
			if(startevent == 1 || yaw_capture_pending != 0U) 
			{ 
                // PD_CONTROL BEGIN：以本次开始时的朝向作为Yaw目标，不重新校准IMU
                yawcontrol = yaw;
                yaw_capture_pending = 0U;
                // PD_CONTROL END
				startevent =0;  
			} 
			 //手柄映射任务 
		   //PS2_Attitude_Control(); 
 /* ============================================================ 
   *激光测高阶段：
1. 判断 startup_calib_done == 1 && startFlag == 1（判断是否无人机启动+零点标记完成）
   ↓ 是
2. 读取 g_readonly_distance（STP23L的数据）
   ↓
3. use_distance = zero_distance - g_readonly_distance（零点减去距离天花板的高度）
   ↓
4. if(use_distance < 0.05f) use_distance = 0.0f（判断相对高度的阈值）
   ↓
5. height_current = use_distance（赋值给真是高度）
   ↓
6. 判断：是第一次？还是距离变了？
   ├─ 第一次（first_frame == 1）
   │     → last_height = height_current（真实高度不变）
   │     → height_speed = 0
   │     → last_height_speed = 0
   │     → first_frame = 0
   │
   └─ 不是第一次，且距离变了
         → raw = (height_current - last_height) / 0.1f （速度=路程/时间）
         → height_speed = 0.4*raw + 0.6*last_height_speed （给高度内环速度加一阶低通滤波）
         → last_height_speed = height_speed（更新高度速度）
         → last_height = height_current（更新高度）
   ↓
7. prev_distance = g_readonly_distance   （保存本次距离，下次判断用）
   ↓
8. 进入高度环 PID+无人机上升阶段(零点标记之后)
 * ============================================================ */ 
if(startup_calib_done ==1)
{
	use_distance =zero_distance -g_readonly_distance;//相对高度
	if(use_distance <0.05f)
	{
		use_distance =0.0f;
	}
	height_current =use_distance;
	
if(first_frame ==1)
{
	      last_height = height_current;
        height_speed = 0.0f;
        last_height_speed = 0.0f;
	
        first_frame = 0;
}
  else if(g_readonly_distance != prev_distance)
{
	  float raw_height_speed;
	
	  raw_height_speed =(height_current -last_height)/0.1f;
	  height_speed =0.4f*raw_height_speed +0.6f*last_height_speed;
	  last_height =height_current;
	  last_height_speed =height_speed;
}
    prev_distance =g_readonly_distance;
}
else
{
	height_speed=0.0f;
	height_current =0.0f;
	use_distance =0.0f;
}


 /* ============================================================ 
   *进入姿态角串级PID+高度PID—+yaw修正（包含gyro的方向判断）
 * ============================================================ */ 
			 Control_PositionHold(&roll_target,&pitch_target);
			 Attitude_PID_Update(roll_target,pitch_target,yawcontrol); //目标姿态角 
            // PD_CONTROL BEGIN：桌面用固定对照油门，不把USB电压代入推力拟合
#if POS_DESKTOP_TEST == 1U
            base_throttle = POS_DESKTOP_BASE;
#else
			 base_throttle=Weight_To_Throttle(target_thrust,battery_voltage); //matlab拟合曲线计算得到的公式
#endif
            // PD_CONTROL END
       //PID_update(&RollPID, 0,roll);//外环Roll 
       //PID_update(&PitchPID, 0,pitch);//外环Pitch 
       //PID_update(&YawPID, 0,yaw);//外环Yaw 
			/*不需要再单独写一遍外环的PID公式，因为已经在Attitude_PID_Update()里面写过了:PID_update(&RollPID,roll_target,roll); PID_update(&PitchPID,pitch_target,pitch);PID_update(&YawPID,yaw_target,yaw);*/ 
			//Yaw方向修正（由于Yaw在转动超过180之后会忽然产生一个峰值，在现实物理世界中，不加入修正，手柄控制yaw的运动飞行器会不自主旋转，所以加入修正） 
      float gx_control;
      float gy_control;
      float gz_control;
      float roll_output;
      float pitch_output;
      float yaw_output;

      gx_control = -gx;
      gy_control = -gy;
      gz_control =  gz;
			
			PID_update(&RollRatePID, RollRateTarget, gx_control );//内环RollRate 
      PID_update(&PitchRatePID, PitchRateTarget, gy_control );//内环PitchRate 
      PID_update(&YawRatePID, YawRateTarget, gz_control );//内环YawRate 

      roll_output = RollRatePID.output;
      pitch_output = PitchRatePID.output;
      yaw_output = YawRatePID.output;
// PD_CONTROL BEGIN：这次桌面位置计算不加入高度修正
#if POS_DESKTOP_TEST == 0U
			PID_update(&HeightPID,height_target,height_current);//外环Height
 /* ============================================================ 
   *高度环限幅（注意：高度外环限幅之后再给高度内环）
 * ============================================================ */ 
      if(HeightPID.output > HEIGHT_SPEED_MAX)
         HeightPID.output = HEIGHT_SPEED_MAX;
			if(HeightPID.output < -HEIGHT_SPEED_MAX) 
				 HeightPID.output = -HEIGHT_SPEED_MAX;
      PID_update(&HeightRatePID,HeightPID.output,height_speed);//内环Height
#else
      PID_Reset(&HeightPID);
      PID_Reset(&HeightRatePID);
#endif
// PD_CONTROL END
 /* ============================================================ 
			*蓝牙QT图像分析（PID参数取舍以及极性）
 * ============================================================ */
			//飞行器最大油门值：1500 实际1148左右 
			//飞行器最小油门值：48 
			//基础油门：matlab拟合曲线之后，10.0最低电源电压之下，基础油门是896，PID总修正是（1500-896）=604 
			//PID修正：这里KD项给100，KP和KI总共给504 
/* ============================================================ 
 * 关于蓝牙图像的测试结果： 
 * 
 角速度(x\y)范围:gyroX轻微摇晃，数值在正负5的区间gyroY轻微摇晃， 
 数值在正负5的区间rollratePID.output 和 pitchratePID.output 预期最大分配500油门值kp估算最大值5*kp=500-->kp的值预估在100以内 
 角速度(z)范围:平衡时基本在小范围内波动gyroZ轻微摇晃， 
 数值在正负1的区间rollratePID.output和 pitchratePID.output 预期最大分配100油门值kp估算最大值1*kp=100->kp的值预估在100以内 
 *  
 角度范围： 
 明确物理意义:外环的输出->物理意义是角速度5或10为最大 
 平衡时的角度(-5~5)5*kp=5或10kp个位数 
 *      
 估算总结: 
 1.内环的kp值在100附近，最大输出值限制在500左右 
 2.外环的kp值在0-5附近，最大输出值限制在正负5区间，如果Z轴，限制约在5以内(取较大的值) 
 * ============================================================ */ 
// HEIGHT_CONTROL：高度内环输出同号加到四路，下面原来的行尾注释保留
#if FLIGHT_MIXER_MODE == 1
m1.throttle = base_throttle - roll_output - pitch_output + yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m2.throttle = base_throttle - roll_output + pitch_output - yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m3.throttle = base_throttle + roll_output + pitch_output + yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m4.throttle = base_throttle + roll_output - pitch_output - yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
#else
m1.throttle = base_throttle + roll_output + pitch_output + yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m2.throttle = base_throttle + roll_output - pitch_output - yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m3.throttle = base_throttle - roll_output - pitch_output + yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
m4.throttle = base_throttle - roll_output + pitch_output - yaw_output + HeightRatePID.output;//+HeightRatePID.output; 
#endif
/* ============================================================ 
*串口打印日志（包含：姿态角+修正的Yaw/油门值+角度环串级PID输出output）
 * ============================================================ */	
// print_cnt++;

// if(print_cnt >= CONTROL_RASK_FREQ / 10)
// {
//    print_cnt = 0;

//    if(xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
//    {
//        printf("ATT | CAL=%d | R=%.3f P=%.3f Y=%.3f | YT=%.3f\r\n",
//               startup_calib_done,
//               roll *57.2957795f,
//               pitch *57.2957795f,
//               yaw *57.2957795f,
//               yawcontrol *57.2957795f);
//
    //    printf("MOTOR | "
    //   "BASE=%u | "
    //   "PID_R=%.2f PID_P=%.2f PID_Y=%.2f | "
    //   "M1=%d M2=%d M3=%d M4=%d\r\n",
    //   base_throttle,
    //   RollRatePID.output,
    //   PitchRatePID.output,
    //   YawRatePID.output,
    //   m1.throttle,
    //   m2.throttle,
    //   m3.throttle,
    //   m4.throttle);

//        xSemaphoreGive(g_xMutex_Printf);
//    }
// }
/* ============================================================ 
			*Roll和Pitch限幅（角度制在40°以下）
 * ============================================================ */ 
			if(fabsf(roll)>0.69813170f||fabsf(pitch)>0.69813170f) 
			{ 
				startFlag =0; 
			} 
/* ============================================================ 
			*油门限制幅度
 * ============================================================ */	
  
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

// HEIGHT_DIAG BEGIN：每秒看5次，不影响原来的姿态和位置参数
//高度用mm，速度用mm/s；H_OUT_x1000除以1000才是加入混控的油门修正
//M1到M4是限幅后的命令，不是电机实际转速；这里只用整数打印
print_cnt++;
if(print_cnt >= CONTROL_RASK_FREQ / 5)
{
    print_cnt = 0;
    if(xSemaphoreTake(g_xMutex_Printf,0U) == pdTRUE)
    {    
           //打印高度环PID输出和油门分配，观察位置环是否成功
//         printf("HEIGHT | T_mm=%ld H_mm=%ld E_mm=%ld V_mm_s=%ld VT_mm_s=%ld H_OUT_x1000=%ld | "
//                "BASE=%u M1=%d M2=%d M3=%d M4=%d\r\n",
//                (long)(height_target*1000.0f),(long)(height_current*1000.0f),
//                (long)((height_target-height_current)*1000.0f),
//                (long)(height_speed*1000.0f),(long)(HeightPID.output*1000.0f),
//                (long)(HeightRatePID.output*1000.0f),(unsigned int)base_throttle,
//                (int)m1.throttle,(int)m2.throttle,(int)m3.throttle,(int)m4.throttle);
// PD_CONTROL BEGIN
          //打印高度环PID输出和油门分配，观察位置环是否成功 
          // 关于control_pos.mode：关闭、1等待、2已定点、3故障锁存；附加倾角用mrad
          printf("POS_MOTOR mode=%u Radd_mrad=%ld Padd_mrad=%ld BASE=%u M1=%d M2=%d M3=%d M4=%d\r\n",
               (unsigned int)control_pos.mode,
               (long)(control_pos.roll_add*1000.0f),(long)(control_pos.pitch_add*1000.0f),
               (unsigned int)base_throttle,
               (int)m1.throttle,(int)m2.throttle,(int)m3.throttle,(int)m4.throttle);
// PD_CONTROL END
        xSemaphoreGive(g_xMutex_Printf);
    }
}
// HEIGHT_DIAG END
		} 
		 else 
    { 
        /* 没有正式启动控制时，保持停止 */ 
        Control_PosReset();
        PID_Reset(&RollPID); 
        PID_Reset(&PitchPID); 
        PID_Reset(&YawPID); 
        PID_Reset(&RollRatePID); 
        PID_Reset(&PitchRatePID); 
        PID_Reset(&YawRatePID); 
		PID_Reset(&HeightPID);
	    PID_Reset(&HeightRatePID);
			
			  //回到最低油门 
        m1.throttle = 48; 
        m2.throttle = 48; 
        m3.throttle = 48; 
        m4.throttle = 48; 
			  //等待下次写入 
			  startevent =1; 
			  //高度数据设置成0
        height_current = 0.0f;
        height_speed   = 0.0f;
		first_frame = 1;
        prev_distance = 0.0f;
    } 
		
//PS2_Throttle_Control();手柄控制检测电机 
//电机转动 
	    //m1.throttle=120; 
		//m2.throttle=120; 
		//m3.throttle=120; 
		//m4.throttle=120; 
		} 
// PD_CONTROL BEGIN：桌面模式禁止电机初始化和发送
#if POS_DESKTOP_TEST == 0U
	//d_shot->set_target(m3,m4,m1,m2);//物理世界映射：A,D,C,B -> 3,4,1,2 
#endif
// PD_CONTROL END
		vTaskDelayUntil(&pretime,pdMS_TO_TICKS(1000.0f/CONTROL_RASK_FREQ)); 
	} 
} 

 
/*============================================================ 
 * 向四轴控制队列写入控制指令（覆盖式写入队列） 
 * writeEnv: 
 * 0 = 普通任务环境 
 * 1 = ISR环境 
*这个检测代码旨意在当出现新的最高优先级的任务时，才会进入队列 
 *============================================================*/ 
void WriteFlyControlQueue(FlyControlType_t val, 
                          uint8_t writeEnv,//判断是否是中断 
                          BaseType_t *HigherPriorityTask) 
{ 
    if(val.source > controlCmdNumber) 
    { 
        controlCmdNumber = val.source; 
    } 
 
    if(val.source == controlCmdNumber) 
    { 
        if(writeEnv == 1) 
        { 
            xQueueOverwriteFromISR(g_xQueueFlyControl,//中断任务 
                                   &val, 
                                   HigherPriorityTask);//平常是xQueueSend排队，现在是 xQueueOverwrite覆盖式写入 
        } 
        else 
        { 
            xQueueOverwrite(g_xQueueFlyControl, &val);//普通任务 
        } 
    } 
} 

