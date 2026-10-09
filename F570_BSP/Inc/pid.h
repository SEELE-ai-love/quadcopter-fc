#ifndef __PID_H
#define __PID_H

#include <stdint.h>


typedef struct{
	float kp; // 比例增益
	float ki; // 积分增益
	float kd; // 微分增益
	float prev_error; // 上一次误差
	float intergral;  // 积分项
	float derivative; //微分项
	float output;     // 输出
	float LimitIntegralMax; //积分上限
	float LimitIntegralMin; //积分下限
	float LimitOutputMax;   //输出上限
	float LimitOutputMin;   //输出下限
	float IntegralThreshold;//积分阈值
	float alpha;            //一阶低通滤波系数
}PIDControllerType_t;

void PID_update(PIDControllerType_t *pid,float target,float current);
void PID_Reset(PIDControllerType_t *pid);
/* ============================================================
 * 【新增】角度环 PID
 *
 * 角度环是外环
 *
 * 输入：
 *     目标角度
 *     当前角度
 *
 * 输出：
 *     目标角速度
 * ============================================================ */

extern PIDControllerType_t RollPID;
extern PIDControllerType_t PitchPID;
extern PIDControllerType_t YawPID;


/* ============================================================
 * 【新增】角速度环 PID
 *
 * 角速度环是内环
 *
 * 输入：
 *     目标角速度
 *     当前角速度
 *
 * 输出：
 *     最终控制量
 * ============================================================ */

extern PIDControllerType_t RollRatePID;
extern PIDControllerType_t PitchRatePID;
extern PIDControllerType_t YawRatePID;;


/* 【新增】高度环 */
extern PIDControllerType_t HeightPID;
extern PIDControllerType_t HeightRatePID;
/* 【新增】只做角度外环 */
void Attitude_PID_Update(float roll_target,
                         float pitch_target,
                         float yaw_target);
//show_task图像，图像曲线调试
extern float RollRateTarget;
extern float PitchRateTarget;
extern float YawRateTarget;
#endif

