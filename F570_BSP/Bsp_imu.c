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
IMU_DATA_T ZreoPoint ={0};//传感器自身固定零偏-
ATTITUDE_DATA_T ZeroAttitude = { -0.17f,1.82f,2.16f};//传感器+机架安装角度偏差(改正方向后重测)

/*
 * ============================================================
 * 初始化驱动ICM20948
 * ============================================================
 */

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

    if (whoami != 0xEA)
    {
        return 0;
    }

    I2C_WriteReg(hi2c,ICM20948_DEV, USER_CTRL, 0x00);//清空功能位
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

    return 1;
}


//读取ICM的值
//【修复】原来读失败时没有return，会继续用未初始化的buf数组算出垃圾数据；
//现在读失败直接return 0，调用者据此决定跳过这一次姿态更新
uint8_t ICM20948_Update(IMU_DATA_T*data,I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[12];

    if (I2C_ReadRegs(hi2c,ICM20948_DEV, ACCEL_XOUT_H, buf, 12) != HAL_OK)
    {
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
