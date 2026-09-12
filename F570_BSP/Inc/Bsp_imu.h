#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "i2c.h"
#include <stdint.h>

extern float Kp;
extern float KI;
typedef struct {
    float x, y, z;
} XYZ_FLOAT_t;

typedef struct {
    XYZ_FLOAT_t accel;
    XYZ_FLOAT_t gyro;
} IMU_DATA_T;

typedef struct {
    float roll, pitch, yaw;
} ATTITUDE_DATA_T;

typedef struct {
    float q0, q1, q2, q3;
} Quaternion_t;

extern ATTITUDE_DATA_T ZeroAttitude;
extern Quaternion_t attitude;

//【修复】返回值从void改成uint8_t，1=成功，0=失败，调用者必须检查
uint8_t ICM20948_Init(I2C_HandleTypeDef *hi2c);
uint8_t ICM20948_Update(IMU_DATA_T *data, I2C_HandleTypeDef *hi2c);

void Quaternation_Update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt, Quaternion_t *q);
void Quaternion_ToEuler(const Quaternion_t *q, float *roll, float *pitch, float *yaw);

#endif
													