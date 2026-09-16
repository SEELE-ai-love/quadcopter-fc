#include "SerialPlot.h"

// =============================================
// JustFloat 协议实现（二进制，高效）[reference:6]
// =============================================
#ifdef VOFA_PROTOCOL_JUSTFLOAT

// 用于打包数据的联合体，方便将float拆解为字节[reference:7]
typedef union {
    float f_data;
    uint32_t u_data;
} Vofa_Type;

// 发送缓冲区
static uint8_t tx_frame[VOFA_FRAME_LEN];

/**
 * @brief 将浮点数拆分为4个字节 (小端模式) [reference:8]
 */
static void float_to_bytes(float value, uint8_t *bytes) {
    Vofa_Type v;
    v.f_data = value;
    bytes[0] = (uint8_t)(v.u_data >> 0);
    bytes[1] = (uint8_t)(v.u_data >> 8);
    bytes[2] = (uint8_t)(v.u_data >> 16);
    bytes[3] = (uint8_t)(v.u_data >> 24);
}

/**
 * @brief 发送三个浮点数到 VOFA+ (JustFloat协议) [reference:9]
 */
void Vofa_Send(float data1, float data2, float data3) {
    uint16_t idx = 0;

    // 1. 将三个float按顺序填入缓冲区
    float_to_bytes(data1, &tx_frame[idx]); idx += 4;
    float_to_bytes(data2, &tx_frame[idx]); idx += 4;
    float_to_bytes(data3, &tx_frame[idx]); idx += 4;

    // 2. 附加 JustFloat 固定帧尾 [reference:10]
    tx_frame[idx++] = VOFA_TAIL0;
    tx_frame[idx++] = VOFA_TAIL1;
    tx_frame[idx++] = VOFA_TAIL2;
    tx_frame[idx++] = VOFA_TAIL3;

    // 3. 通过串口发送 (使用阻塞或DMA方式)
    //    推荐使用DMA发送以提高效率[reference:11]
    HAL_UART_Transmit(&huart1, tx_frame, VOFA_FRAME_LEN, 100);
    // HAL_UART_Transmit_DMA(&huart1, tx_frame, VOFA_FRAME_LEN);
}

#endif

// =============================================
// FireWater 协议实现（文本，易读）[reference:12]
// =============================================
#ifdef VOFA_PROTOCOL_FIREWATER

#include <stdio.h> // for sprintf

/**
 * @brief 发送三个浮点数到 VOFA+ (FireWater协议)
 *        格式: "value1,value2,value3\n" [reference:13]
 */
void Vofa_Send(float data1, float data2, float data3) {
    char buffer[64];
    int len = sprintf(buffer, "%.3f,%.3f,%.3f\n", data1, data2, data3);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

#endif

// =============================================
// 初始化 (若需要)
// =============================================
void Vofa_Init(void) {
    // 如果使用DMA发送，可以在这里做一些初始化
    // 目前留空
}