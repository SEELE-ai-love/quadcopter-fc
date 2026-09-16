#include "Bsp_imu.h"
#include "Bsp_iic.h"
#include <math.h>
#include <stdio.h>
#include "icm20948_reg.h"

// ==========================================
// 读取加速度原始值，直接算角度
// ==========================================
#define radian 57.2958f
#define EINT_LIMIT 0.3f
//加速度计可信区间判断
#define ACCEL_TRUST_LOW 0.85f
#define ACCEL_TRUST_HIGH 1.15f
//定义全局变量
static float eInt[3] = {0.0f, 0.0f, 0.0f};//存放积分误差
IMU_DATA_T ZreoPoint ={0};//传感器自身固定零偏-
ATTITUDE_DATA_T ZeroAttitude ={-0.93f, 0.95f, 0.0f};//传感器安装角度偏差
float Kp=0.8f;
float Ki=0.0001f;
//使用算法：牛顿迭代计算1/sqrt
static float insprt(float x)
{
	float halfx=0.5f*x;
	float y=x;
	long i=*(long *)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-halfx *y*y);
	return y;
}
//初始化驱动ICM20948
//【修复】原来没有WHOAMI校验、复位后没等待、返回类型是void——现在改成uint8_t返回，
//调用者必须检查返回值，1=成功，0=失败（芯片没接好/地址不对，不能继续往下跑）
uint8_t ICM20948_Init(I2C_HandleTypeDef*hi2c)
{
	uint8_t whoami = 0;

	I2C_WriteReg(hi2c,ICM20948_DEV,PWR_MGMT_1,0x80);//复位整个芯片
	HAL_Delay(100);//【修复】之前这里被删掉了，复位需要时间，不等待的话后面的配置可能没生效

	//【修复】加回WHOAMI校验，确认芯片真的连上了，不然出问题都不知道是不是通信没建立
	I2C_ReadReg(hi2c, ICM20948_DEV, WHO_AM_I, &whoami);
	if (whoami != 0xEA) {
		printf("ICM20948 not found! whoami=0x%02X\r\n", whoami);
		return 0;
	}

	I2C_WriteReg(hi2c, ICM20948_DEV, USER_CTRL, 0x00);//清空功能位
	I2C_WriteReg(hi2c,ICM20948_DEV, PWR_MGMT_1,0x01);//唤醒芯片+开启时钟
	I2C_WriteReg(hi2c, ICM20948_DEV, PWR_MGMT_2, 0x00);   // 加速度计和陀螺仪全部使能
	I2C_WriteReg(hi2c, ICM20948_DEV, REG_BANK_SEL, REG_VAL_SELECT_BANK_2);//切换Bank2
	I2C_WriteReg(hi2c, ICM20948_DEV, GYRO_SMPLRT_DIV, 0x04);//陀螺仪输出速率 1100/(1+4)=220Hz
	I2C_WriteReg(hi2c, ICM20948_DEV, GYRO_CONFIG_1, (3 << 1) | (1 << 0) | (3 << 3));//
	//【修复】加回高8位分频寄存器，之前只写了低8位(ACCEL_SMPLRT_DIV_2)，高8位保持默认值可能不是0
	I2C_WriteReg(hi2c, ICM20948_DEV, ACCEL_SMPLRT_DIV_1, 0x00);
	I2C_WriteReg(hi2c, ICM20948_DEV, ACCEL_SMPLRT_DIV_2, 0x04);//加速度计输出速率 1125/(1+4)=225Hz
	I2C_WriteReg(hi2c, ICM20948_DEV, ACCEL_CONFIG, (0 << 1) | (1 << 0) | (5 << 3));//
	I2C_WriteReg(hi2c, ICM20948_DEV, REG_BANK_SEL, REG_VAL_SELECT_BANK_0);//切回Bank0
	//I2C_WriteReg(hi2c, ICM20948_DEV, INT_PIN_CFG, (1 << 1));// 开I2C旁路，暴露AK09916
  //I2C_WriteReg(hi2c, AK09916_DEV, CNTL2, (1 << 3));// 磁力计：连续模式4，100Hz

	printf("ICM20948_init OK!\r\n");
	return 1;
}

//读取ICM的值
//【修复】原来读失败时没有return，会继续用未初始化的buf数组算出垃圾数据；
//现在读失败直接return 0，调用者据此决定跳过这一次姿态更新
uint8_t ICM20948_Update(IMU_DATA_T*data,I2C_HandleTypeDef *hi2c)
{
	uint8_t buf[12];
	if (I2C_ReadRegs(hi2c,ICM20948_DEV, ACCEL_XOUT_H, buf, 12) != HAL_OK) {
		return 0;   //【修复】之前这里只打印了"I2C read failed"但没有return
	}

	data->accel.x =(short)(buf[0]<<8)|buf[1];
	data->accel.y =(short)(buf[2]<<8)|buf[3];
	data->accel.z =(short)(buf[4]<<8)|buf[5];

	data->accel.x *= 0.00059814453125f;
	data->accel.y *= 0.00059814453125f;
	data->accel.z *= 0.00059814453125f;

	data->gyro.x =(short)(buf[6]<<8)|buf[7];
	data->gyro.y =(short)(buf[8]<<8)|buf[9];
	data->gyro.z =(short)(buf[10]<<8)|buf[11];

	data->gyro.x *= 0.06103515625f;   // LSB → dps
	data->gyro.y *= 0.06103515625f;
	data->gyro.z *= 0.06103515625f;

	data->gyro.x *= 0.01745329252f;   // dps → rad/s
	data->gyro.y *= 0.01745329252f;
	data->gyro.z *= 0.01745329252f;
	
	data->gyro.x -= 0.0100f;   
  data->gyro.y += 0.0021f;
	data->gyro.z += 0.0020f;  

	return 1;
}

// ============================================================
// 四元数基础运算（陀螺仪积分，Mahony互补滤波加速度修正）
// ============================================================

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle_t;


// 全局四元数（初始化为单位四元数）
Quaternion_t attitude = {1.0f, 0.0f, 0.0f, 0.0f};

/**
 * @brief 陀螺仪积分更新四元数（基础版，无加速度修正）
 * @param gx, gy, gz  陀螺仪角速度（rad/s）
 * @param dt          采样周期（秒）
 * @param q           四元数结构体指针
 */
void Quaternation_Update(float gx,float gy,float gz,float ax,float ay,float az,float dt,Quaternion_t*q)//四元数运算+Mahony互补滤波+计算姿态角
{
	//更新四元数
	float q0=q->q0;
	float q1=q->q1;
	float q2=q->q2;
	float q3=q->q3;
	
	//归一化加速度计
  float norm_a = sqrtf(ax*ax + ay*ay + az*az);
  if(norm_a >=ACCEL_TRUST_LOW &&norm_a <=ACCEL_TRUST_HIGH)
	{
  if (norm_a > 1e-12f) {
    ax /= norm_a;
    ay /= norm_a;
    az /= norm_a;
   } else {
    ax = 0.0f; ay = 0.0f; az = 0.0f;
   }
   }

	//反推理论重力
	float vx=2.0f*(q1*q3-q0*q2);
	float vy=2.0f*(q0*q1+q2*q3);
	float vz=1.0f-2.0f*(q1*q1+q2*q2);

	//叉乘算误差
	float ex=ay*vz-az*vy;
	float ey=az*vx-ax*vz;
	float ez=ax*vy-ay*vx;
	
	//积分误差，对误差进叠加
	eInt[0] +=ex;
	eInt[1] +=ey;
	eInt[2] +=ez;


//float Kp = 0.8f;
//float Ki = 0.0001f;
gx += Kp * ex + Ki * eInt[0];
gy += Kp * ey + Ki * eInt[1];
gz += Kp * ez + Ki * eInt[2];


	float halft=0.5f*dt;
	//四元数矩阵微分运算
	//
	 float q0_dot = (-q1 * gx - q2 * gy - q3 * gz) * halft;
   float q1_dot = ( q0 * gx + q2 * gz - q3 * gy) * halft;
   float q2_dot = ( q0 * gy - q1 * gz + q3 * gx) * halft;
   float q3_dot = ( q0 * gz + q1 * gy - q2 * gx) * halft;

//更新四元数

	q0+=q0_dot;
	q1+=q1_dot;
	q2+=q2_dot;
	q3+=q3_dot;

//四元数归一化
float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
if (norm > 1e-12f) {
    q->q0 = q0 / norm;
    q->q1 = q1 / norm;
    q->q2 = q2 / norm;
    q->q3 = q3 / norm;
} else {
    q->q0 = 1.0f; q->q1 = 0.0f; q->q2 = 0.0f; q->q3 = 0.0f;
}


}

//四元数——>欧拉角
void Quaternion_ToEuler(const Quaternion_t *q,float *roll,float *pitch,float *yaw)
{
	float q0=q->q0;
	float q1=q->q1;
	float q2=q->q2;
	float q3=q->q3;

//Roll是滚转角，Pitch是俯仰角，Row是偏航角（这个需要磁力计介入）
	* roll =atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2))*radian;
	* pitch =asinf(2.0f * (q0 * q2 - q3 * q1))*radian;
	* yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3))*radian;
	
	*roll  -= ZeroAttitude.roll;   // 减负数 = 加 0.93
  *pitch -= ZeroAttitude.pitch;  // 减正数 = 减 0.95
  *yaw   -= ZeroAttitude.yaw;    //不变
}
