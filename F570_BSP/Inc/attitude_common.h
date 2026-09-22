#ifndef __ATTITUDE_COMMON_H
#define __ATTITUDE_COMMON_H

/*
 * ============================================================
 * 姿态算法公共数据结构
 * ============================================================
 */

/* 四元数结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

} Quaternion_t;

#endif
