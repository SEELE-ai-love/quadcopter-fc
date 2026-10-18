#ifndef BSP_LC307_H
#define BSP_LC307_H

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"


#ifdef __cplusplus
extern "C" {
#endif
/* DMA 接收缓冲区容量,这里是HAL_UARTEx_ReceiveToIdle_DMA（）手动启动DMA搬运的第三个参数 */
#define userconfig_LC307_DMABUF_LEN  50U
#define LC307_FRAME_LEN              14U

typedef struct
{
    uint16_t DataLen;
    uint8_t  Buf[userconfig_LC307_DMABUF_LEN];
    TickType_t RxTick;//本轮DMA接收结束的时间
    uint32_t RxEpoch;//接收故障代数，用来丢弃故障之前的旧数据块
} DMABuf_oridata_LC307_t;

extern DMABuf_oridata_LC307_t DMABuf_oridata_LC307;

/* 仅由 LC307 UART ISR 累加；定义位于 uart_callback.c。 */
extern volatile uint32_t g_lc307_queue_overflow_count;

#define LC307_RX_REQUEST_START    0x01U
#define LC307_RX_REQUEST_RECOVER  0x02U
//中断和启动任务只提出请求，只有光流任务操作DMA的启动和中止
extern volatile uint32_t g_lc307_rx_epoch;
extern volatile uint8_t g_lc307_rx_requests;
extern uint8_t g_lc307_config_ok;

typedef struct
{
    uint8_t  head1;
    uint8_t  bufcount;
    int16_t  flowX;
    int16_t  flowY;
    uint16_t timespan;
    uint16_t distance;
    uint8_t  quality;
    uint8_t  version;
    uint8_t  XORSum;
    uint8_t  end;
} OpticalFlowFrame_t;

/* 在 main() 中调用：给 LC307 发送配置。 */
uint8_t Opf_LC307_Init(void);

/* 供光流任务调用：检查并解析一帧，成功返回 1。 */
uint8_t LC307_ParseFrame(const uint8_t *buffer,
                         uint16_t size,
                         OpticalFlowFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
