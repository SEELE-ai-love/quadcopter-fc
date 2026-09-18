#ifndef __BSP_DSHOT_H
#define __BSP_DSHOT_H

#include <stdint.h>

/* ?? DShot命令区（0~47），发送命令时需把Telemetry置1（遥测位置1） */
enum
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1, DSHOT_CMD_BEEP2, DSHOT_CMD_BEEP3, DSHOT_CMD_BEEP4, DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1, DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF, DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, DSHOT_CMD_LED1_ON, DSHOT_CMD_LED2_ON, DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF, DSHOT_CMD_LED1_OFF, DSHOT_CMD_LED2_OFF, DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_MAX = 47
};

/* ?? 电机数据结构：注意顺序 Telemetry 在前，throttle 在后 */
typedef struct
{
    uint8_t Telemetry;
    short   throttle;
} dshotMotorVal_t;

/*  驱动接口（函数指针结构体） */
typedef struct
{
    void (*init)(void);
    void (*set_target)(dshotMotorVal_t m1, dshotMotorVal_t m2,
                        dshotMotorVal_t m3, dshotMotorVal_t m4);
} MotorInterface_t, *pMotorInterface_t;//定义两个类型，一个是结构体，一个是指针类型的结构体

extern MotorInterface_t UserDshotMotor;//UserDshotMotor包含init和set_target的结构

#endif /* __BSP_DSHOT_H */
