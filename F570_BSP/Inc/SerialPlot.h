#ifndef __SERIALPLOT_H
#define __SERIALPLOT_H

#include "usart.h"
#include <stdint.h>
#include <string.h>

// =============================================
// 协议选择：只需定义其中一个
// =============================================
#define VOFA_PROTOCOL_JUSTFLOAT  1   // 二进制协议，高效
//#define VOFA_PROTOCOL_FIREWATER  1  // 文本协议，易读

// =============================================
// JustFloat 协议配置
// =============================================
#ifdef VOFA_PROTOCOL_JUSTFLOAT
    #define VOFA_CH_NUM     3       // 通道数 (例如: roll, pitch, yaw)
    #define VOFA_TAIL_LEN   4       // 帧尾长度
    #define VOFA_FRAME_LEN  (VOFA_CH_NUM * 4 + VOFA_TAIL_LEN) // 总帧长

    // JustFloat 固定帧尾: 0x00, 0x00, 0x80, 0x7F [reference:4][reference:5]
    #define VOFA_TAIL0      0x00
    #define VOFA_TAIL1      0x00
    #define VOFA_TAIL2      0x80
    #define VOFA_TAIL3      0x7F
#endif

// =============================================
// API 函数
// =============================================
void Vofa_Init(void);                              // 初始化（如DMA或中断）
void Vofa_Send(float data1, float data2, float data3); // 发送三个浮点数

#endif