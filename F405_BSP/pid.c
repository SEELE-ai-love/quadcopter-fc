#include "pid.h"
#include "attitude_algorithm.h"
#include "math.h"
#include <stdio.h>

#define DEG_TO_RAD 0.017453292519943f //在飞控中1度等于π/180，这里用来把角度转换成弧度

//无人机工程的三层串级PID蓝图：
//             目标姿态
//                │
//       ┌────────┼────────┐
//       ↓        ↓        ↓
//     Roll     Pitch     Yaw
//       │        │        │
//       ↓        ↓        ↓
//     角度环    角度环    角度环
//       │        │        │
//       ↓        ↓        ↓
//    目标角速  目标角速  目标角速
//       │        │        │
//       ↓        ↓        ↓
//    角速度环  角速度环  角速度环
//       │        │        │
//       └────────┼────────┘
//                ↓
//              混控
//                ↓
//        ┌───────┼───────┐
//    ↓       ↓       ↓       ↓
//    M1      M2      M3      M4
//注意：角速度环是内环，是底层，它产出角速度驱动电机混控，然后角度环是外环，它产生角速度给角速度环，角速度再驱动电机混控，角速度环是重点
void PID_Reset(PIDControllerType_t *pid)
{
	pid->prev_error=0.0f;//上一次误差
	pid ->intergral=0.0f;//PID公式积分项，KI后面的数
	pid ->output =0.0f;//PID最后的输出结果
	pid ->derivative=0.0f;//PID公式微分项，Kd后面的数
}

//关于变量单位：由于双环PID如果变量单位不一样会导致严重的后果，所以统一规定单位是°/s

//| 项目           | 现在单位  |
//| ------------   | ----- |
//| accel          | m/s2  |
//| gyro           | rad/s |
//| Mahony/EKF输入 | rad/s |
//| Euler Roll     | rad   |
//| Euler Pitch    | rad   |
//| Euler Yaw      | rad   |
//| 外环输入       | rad   |
//| 外环输出       | rad/s |
//| Rate Target    | rad/s |
//| Rate PID 输入  | rad/s |
//| Mixer输出      |控制量 |
//| Yaw step       | rad   |

// Yaw修正，将角度误差限制到 [-180°, 180°]
// 保证 Yaw PID 永远选择最近的转动方向
static float Angle_Error(float target, float current)
{
    float error = target - current;
    while (error > 3.14159265f)
    {
        error -= 6.28318531f;
    }
    while (error < -3.14159265f)
    {
        error += 6.28318531f;
    }
    return error;
}

void PID_update(PIDControllerType_t *pid,float target,float current)//PID_update的目的是生成output
{
	float error;
	float Tmpdenvative;
	error =target -current;
	//积分项
	if(fabsf(error) <pid ->IntegralThreshold)//积分阈值，达到最小积分才开始积分
	{
		pid->intergral +=error;
	}
	//积分限幅
	if(pid->intergral >pid->LimitIntegralMax)
	{
		pid->intergral =pid->LimitIntegralMax;
	}
		if(pid->intergral <pid->LimitIntegralMin)
	{
		pid->intergral =pid->LimitIntegralMin;
	}
	//微分项
  //	pid->derivative =error -pid->prev_error;
	//更改微分项，换成微分低通滤波，给微分项滤波，因为KD项容易受到噪声的干扰
	Tmpdenvative =error -pid->prev_error;
	pid->derivative =pid->alpha *Tmpdenvative +(1-pid->alpha )*pid->derivative;
	//更新历史误差
	pid->prev_error =error;
	//没有微分限幅，只有微分先行
	
	//PID输出output
	pid->output=pid->kp *error+pid->ki*pid->intergral+pid->kd*pid->derivative;
	//输出限幅
	if(pid->output >pid->LimitOutputMax)
	{
		pid->output =pid->LimitOutputMax;
	}
		if(pid->output <pid->LimitOutputMin)
	{
		pid->output =pid->LimitOutputMin;
	}
}
	
	/* ============================================================
 * 【新增】
 * 第一层：角度外环
 *
 * Roll / Pitch / Yaw
 *
 * 输出：
 *     目标角速度
 * ============================================================ */
PIDControllerType_t RollPID =    //滚转Roll角度环
{
    .kp = 6.0f,
	 // .kp=0.0f,
    .ki = 0.0f,
    .kd = 0.0f,

    .IntegralThreshold = 0.2f,

    .LimitIntegralMax = 0.52f,
    .LimitIntegralMin = -0.52f,

    .LimitOutputMax = 1.7f,
    .LimitOutputMin = -1.7f,

    .alpha = 0.9f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};

PIDControllerType_t PitchPID =    //俯仰Pitch角度环
{
    .kp = 6.0f,
	  //.kp=0.0f,
    .ki = 0.0f,
    .kd = 0.0f,

    .IntegralThreshold = 0.2f,

    .LimitIntegralMax = 0.52f,
    .LimitIntegralMin = -0.52f,

    .LimitOutputMax = 1.7f,
    .LimitOutputMin = -1.7f,

    .alpha = 0.9f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};

PIDControllerType_t YawPID =     //偏航Yaw角度环
{
    .kp = 6.0f,
	  //.kp=0.0f,
    .ki = 0.0f,
    .kd = 1.5f,

    .IntegralThreshold = 0.2f,

    .LimitIntegralMax = 0.52f,
    .LimitIntegralMin = -0.52f,

    .LimitOutputMax = 1.7f,
    .LimitOutputMin = -1.7f,

    .alpha = 1.0f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};

/*============================================================
 * 角速度环目标
 *
 * 单位统一规定：
 * RollRateTarget  = rad/s
 * PitchRateTarget = rad/s
 * YawRateTarget   = rad/s
 *============================================================*/
float RollRateTarget = 0.0f;
float PitchRateTarget = 0.0f;
float YawRateTarget = 0.0f;

void Attitude_PID_Update(float roll_target,
                         float pitch_target,
                         float yaw_target)
{
    float roll;
    float pitch;
    float yaw;

    /* ========================================================
     * 【新增】
     * 从你现有的姿态算法模块获取姿态角
     * 这里就是EKF输出链。
     * ======================================================== */
    Attitude_GetEuler(&roll, &pitch, &yaw);
	  /*Yaw修正：如果目标角速度减去实际角速度大于180，就减360，反之加360*/
	  float Yaw_error=Angle_Error(yaw_target,yaw);
    /* ========================================================
     * 角度外环
     * 目标角度 → 当前EKF姿态角
     * 输出：
     *     目标角速度
	   * 对yaw的修正比较特殊，需要归一化
     * ======================================================== */
    PID_update(&RollPID,
               roll_target,
               roll);
    PID_update(&PitchPID,
               pitch_target,
               pitch);
		//yaw修正
    PID_update(&YawPID,
               0.0f,
               -Yaw_error);
    /* ========================================================
     * 【新增】
     * 保存角度环输出
     * 这些就是下一阶段角速度环的Target。
     * ======================================================== */
    RollRateTarget = RollPID.output;
    PitchRateTarget = PitchPID.output;
    YawRateTarget = YawPID.output;
}


/* ============================================================
 * 【新增】
 * 第二层：角速度内环
 *
 * Roll / Pitch / Yaw
 *
 * 输出：
 *     最终控制量
 *
 * 这里的输出以后会进入 Mixer，
 * 再由 Mixer 分配给四个电机。
 * ============================================================ */
PIDControllerType_t RollRatePID =
{
    .kp = 85.0f,
     //.kp = 20.0f,
    .ki = 0.2f,
	//.ki=0.0f,
    .kd = 0.5f,

    .IntegralThreshold = 0.35f,

    .LimitIntegralMax = 3.5f,
    .LimitIntegralMin = -3.5f,

    .LimitOutputMax = 700.0f,
    .LimitOutputMin = -700.0f,

    .alpha = 0.8f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};

PIDControllerType_t PitchRatePID =
{
    .kp = 85.0f,
	 //.kp = 20.0f,
    .ki = 0.2f,
		//.ki=0.0f,
    .kd = 0.5f,

    .IntegralThreshold = 0.35f,

    .LimitIntegralMax = 3.5f,
    .LimitIntegralMin = -3.5f,

    .LimitOutputMax = 700.0f,
    .LimitOutputMin = -700.0f,

    .alpha = 0.8f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};
PIDControllerType_t YawRatePID =
{
    .kp = 200.0f,
	//.kp = 30.0f,
    .ki = 1.0f,
		//.ki=0.0f,
    .kd = 10.0f,

    .IntegralThreshold = 0.17f,

    .LimitIntegralMax = 1.04f,
    .LimitIntegralMin = -1.04f,

    .LimitOutputMax = 500.0f,
    .LimitOutputMin = -500.0f,

    .alpha = 0.8f,

    .prev_error = 0.0f,
    .intergral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f
};

//高度内环
PIDControllerType_t HeightRatePID = {
	.kp = 60.0f,
	.ki = 0.02f,
	.kd = 80.0f,
	.LimitIntegralMax = 100.0f,
	.LimitIntegralMin = -100.0f,
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 20 ,
	.alpha = 0.4,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//高度外环
PIDControllerType_t HeightPID = {
	.kp = 3.5f,
	.ki = 0.0f,
	.kd = 0.5f,
	.LimitIntegralMax = 10,
	.LimitIntegralMin = -10,
	.LimitOutputMax = 1.0f,
	.LimitOutputMin = -1.0f,
	.IntegralThreshold = 10 ,
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

