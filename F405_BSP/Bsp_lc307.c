#include "Bsp_LC307.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define LC307_XOR_ERR  1U
#define LC307_ACK_ERR  2U
#define LC307_TIMEOUT  100U

//启动窗口先用100ms，最多试3次；这里只在main里跑，不能放进飞行任务
#define LC307_INIT_ATTEMPTS     3U
#define LC307_INIT_WAIT_MS      100U

typedef struct
{
    const char *stage;
    HAL_StatusTypeDef hal_status;
    uint32_t uart_error;
    uint16_t reg_index;
    uint8_t ack[3];
    uint8_t ack_len;
} LC307_InitDiag_t;

static LC307_InitDiag_t init_diag;

DMABuf_oridata_LC307_t DMABuf_oridata_LC307;
uint8_t g_lc307_config_ok = 0U;//模块配置是否成功，Opf_LC307_Init返回0才是成功

/* 配置表：地址、数据成对排列。 */
static const uint8_t tab_BF3901_60hz[] =
{
    0x12,0x80, 0x11,0x30, 0x1b,0x06, 0x6b,0x43, 0x12,0x20,
    0x3a,0x00, 0x15,0x02, 0x62,0x81, 0x08,0xa0, 0x06,0x68,
    0x2b,0x20, 0x92,0x25, 0x27,0x97, 0x17,0x01, 0x18,0x79,
    0x19,0x00, 0x1a,0xa0, 0x03,0x00, 0x13,0x00, 0x01,0x13,
    0x02,0x20, 0x87,0x16, 0x8c,0x01, 0x8d,0xcc, 0x13,0x07,
    0x33,0x10, 0x34,0x1d, 0x35,0x46, 0x36,0x40, 0x37,0xa4,
    0x38,0x7c, 0x65,0x46, 0x66,0x46, 0x6e,0x20, 0x9b,0xa4,
    0x9c,0x7c, 0xbc,0x0c, 0xbd,0xa4, 0xbe,0x7c, 0x20,0x09,
    0x09,0x03, 0x72,0x2f, 0x73,0x2f, 0x74,0xa7, 0x75,0x12,
    0x79,0x8d, 0x7a,0x00, 0x7e,0xfa, 0x70,0x0f, 0x7c,0x84,
    0x7d,0xba, 0x5b,0xc2, 0x76,0x90, 0x7b,0x55, 0x71,0x46,
    0x77,0xdd, 0x13,0x0f, 0x8a,0x10, 0x8b,0x20, 0x8e,0x21,
    0x8f,0x40, 0x94,0x41, 0x95,0x7e, 0x96,0x7f, 0x97,0xf3,
    0x13,0x07, 0x24,0x58, 0x97,0x48, 0x25,0x08, 0x94,0xb5,
    0x95,0xc0, 0x80,0xf4, 0x81,0xe0, 0x82,0x1b, 0x83,0x37,
    0x84,0x39, 0x85,0x58, 0x86,0xff, 0x89,0x15, 0x8a,0xb8,
    0x8b,0x99, 0x39,0x98, 0x3f,0x98, 0x90,0xa0, 0x91,0xe0,
    0x40,0x20, 0x41,0x28, 0x42,0x26, 0x43,0x25, 0x44,0x1f,
    0x45,0x1a, 0x46,0x16, 0x47,0x12, 0x48,0x0f, 0x49,0x0d,
    0x4b,0x0b, 0x4c,0x0a, 0x4e,0x08, 0x4f,0x06, 0x50,0x06,
    0x5a,0x56, 0x51,0x1b, 0x52,0x04, 0x53,0x4a, 0x54,0x26,
    0x57,0x75, 0x58,0x2b, 0x5a,0xd6, 0x51,0x28, 0x52,0x1e,
    0x53,0x9e, 0x54,0x70, 0x57,0x50, 0x58,0x07, 0x5c,0x28,
    0xb0,0xe0, 0xb1,0xc0, 0xb2,0xb0, 0xb3,0x4f, 0xb4,0x63,
    0xb4,0xe3, 0xb1,0xf0, 0xb2,0xa0, 0x55,0x00, 0x56,0x40,
    0x96,0x50, 0x9a,0x30, 0x6a,0x81, 0x23,0x33, 0xa0,0xd0,
    0xa1,0x31, 0xa6,0x04, 0xa2,0x0f, 0xa3,0x2b, 0xa4,0x0f,
    0xa5,0x2b, 0xa7,0x9a, 0xa8,0x1c, 0xa9,0x11, 0xaa,0x16,
    0xab,0x16, 0xac,0x3c, 0xad,0xf0, 0xae,0x57, 0xc6,0xaa,
    0xd2,0x78, 0xd0,0xb4, 0xd1,0x00, 0xc8,0x10, 0xc9,0x12,
    0xd3,0x09, 0xd4,0x2a, 0xee,0x4c, 0x7e,0xfa, 0x74,0xa7,
    0x78,0x4e, 0x60,0xe7, 0x61,0xc8, 0x6d,0x70, 0x1e,0x39,
    0x98,0x1a, 0x9d,0xf0
};

static uint8_t LC307_CheckAck(const uint8_t feedback[3])
{
    if ((uint8_t)(feedback[0] ^ feedback[1]) != feedback[2])
    {
        return LC307_XOR_ERR;
    }

    if (feedback[1] != 0x00U)
    {
        return LC307_ACK_ERR;
    }

    return 0U;
}

static HAL_StatusTypeDef LC307_ReadConfigAck(uint8_t feedback[3])
{
    memset(feedback,0,3U);
    init_diag.hal_status = HAL_UART_Receive(&huart3,feedback,3U,LC307_TIMEOUT);
    init_diag.uart_error = HAL_UART_GetError(&huart3);
    init_diag.ack_len = (init_diag.hal_status == HAL_BUSY) ? 0U :
                        (uint8_t)(3U-huart3.RxXferCount);
    memcpy(init_diag.ack,feedback,3U);
    return init_diag.hal_status;
}

static uint8_t LC307_InitOnce(void)
{
    static const uint8_t step1_initbuf[7] =
        {0xAA, 0xAB, 0x96, 0x26, 0xBC, 0x50, 0x5C};

    uint8_t feedbackbuf[3];
    uint8_t result;

    /* 这里只配置模块，不启动 DMA；DMA 在 StartDefaultTask 中启动。 */
    init_diag.stage = "prepare";
    init_diag.hal_status = HAL_UART_DMAStop(&huart3);
    if(init_diag.hal_status != HAL_OK) return LC307_ACK_ERR;
    init_diag.hal_status = HAL_UART_AbortReceive(&huart3);
    if(init_diag.hal_status != HAL_OK) return LC307_ACK_ERR;
    //上次残留字节和ORE先清掉；清理有上限，不在这里一直等串口安静
    {
        uint8_t discard;
        for(uint16_t n = 0U; n < 64U; n++)
        {
            if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE) == RESET) break;
            discard = (uint8_t)huart3.Instance->DR;
            (void)discard;
        }
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }

    init_diag.stage = "enter_tx";
    if ((init_diag.hal_status = HAL_UART_Transmit(&huart3, (uint8_t *)step1_initbuf,
                          sizeof(step1_initbuf), LC307_TIMEOUT)) != HAL_OK)
    {
        return LC307_ACK_ERR;
    }

    init_diag.stage = "enter_rx";
    if (LC307_ReadConfigAck(feedbackbuf) != HAL_OK)
    {
        return LC307_ACK_ERR;
    }

    init_diag.stage = "enter_ack";
    result = LC307_CheckAck(feedbackbuf);
    if (result != 0U)
    {
        return result;
    }

    for (uint16_t i = 0U; i < sizeof(tab_BF3901_60hz); i += 2U)
    {
        uint8_t command[5] =
        {
            0xBB,
            0xDC,
            tab_BF3901_60hz[i],
            tab_BF3901_60hz[i + 1U],
            0U
        };

        init_diag.reg_index = i / 2U;
        init_diag.ack_len = 0U;
        memset(init_diag.ack,0,sizeof(init_diag.ack));
        init_diag.stage = "reg_tx";
        command[4] = (uint8_t)(command[1] ^ command[2] ^ command[3]);

        if ((init_diag.hal_status = HAL_UART_Transmit(&huart3, command, sizeof(command),
                              LC307_TIMEOUT)) != HAL_OK)
        {
            return LC307_ACK_ERR;
        }

        init_diag.stage = "reg_rx";
        if (LC307_ReadConfigAck(feedbackbuf) != HAL_OK)
        {
            return LC307_ACK_ERR;
        }

        init_diag.stage = "reg_ack";
        result = LC307_CheckAck(feedbackbuf);
        if (result != 0U)
        {
            return result;
        }
    }

    {
        uint8_t closecfg = 0xDDU;

        init_diag.stage = "exit_tx";
        init_diag.ack_len = 0U;
        memset(init_diag.ack,0,sizeof(init_diag.ack));
        if ((init_diag.hal_status = HAL_UART_Transmit(&huart3, &closecfg, 1U,
                              LC307_TIMEOUT)) != HAL_OK)
        {
            return LC307_ACK_ERR;
        }
    }

    init_diag.stage = "done";
    return 0U;
}

//只做启动配置重试，不启动DMA；进RTOS以后仍由LC307任务独占接收操作
uint8_t Opf_LC307_Init(void)
{
    uint8_t result = LC307_ACK_ERR;
    char log[192];

    for(uint8_t attempt = 1U; attempt <= LC307_INIT_ATTEMPTS; attempt++)
    {
        int length;
        memset(&init_diag,0,sizeof(init_diag));
        init_diag.reg_index = 0xFFFFU;//还没走到寄存器表
        //这是待上机确认的启动等待，不代表模块手册规定的时序
        HAL_Delay(LC307_INIT_WAIT_MS);
        result = LC307_InitOnce();
        //先留住USART3状态，再用USART1输出；这里调度器还没启动，不拿RTOS互斥锁
        //打印值来验证为什么每一次烧录之后会有不同的结果，或者为什么有时会失败
        init_diag.uart_error = HAL_UART_GetError(&huart3);
        length = snprintf(log,sizeof(log),
                          "LC307 INIT try=%u result=%u stage=%s reg=%u hal=%u err=%lu ack_n=%u ack=%02X,%02X,%02X\r\n",
                          (unsigned int)attempt,(unsigned int)result,init_diag.stage,
                          (unsigned int)init_diag.reg_index,(unsigned int)init_diag.hal_status,
                          (unsigned long)init_diag.uart_error,(unsigned int)init_diag.ack_len,
                          (unsigned int)init_diag.ack[0],(unsigned int)init_diag.ack[1],
                          (unsigned int)init_diag.ack[2]);
        if(length > 0)
        {
            uint16_t size = (length < (int)sizeof(log)) ? (uint16_t)length :
                            (uint16_t)(sizeof(log)-1U);
            (void)HAL_UART_Transmit(&huart1,(uint8_t *)log,size,LC307_TIMEOUT);
        }
        if(result == 0U) break;
    }
    //失败仍然返回非0，main保持cfg=0，不让无效数据进入位置环
    return result;
}

static uint8_t LC307_XorChecksum(const uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0U;

    for (uint16_t i = 0U; i < length; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}

uint8_t LC307_ParseFrame(const uint8_t *buffer,
                         uint16_t size,
                         OpticalFlowFrame_t *frame)
{
    if ((buffer == NULL) || (frame == NULL) || (size < LC307_FRAME_LEN))
    {
        return 0U;
    }

    for (uint16_t i = 0U; i <= size - LC307_FRAME_LEN; i++)
    {
        const uint8_t *p = &buffer[i];

        if (p[0] != 0xFEU)
        {
            continue;
        }

        if (p[12] != LC307_XorChecksum(&p[2], 10U))
        {
            continue;
        }

        frame->head1    = p[0];
        frame->bufcount = p[1];
        frame->flowX    = (int16_t)((uint16_t)p[2] |
                                    ((uint16_t)p[3] << 8));
        frame->flowY    = (int16_t)((uint16_t)p[4] |
                                    ((uint16_t)p[5] << 8));
        frame->timespan = (uint16_t)((uint16_t)p[6] |
                                     ((uint16_t)p[7] << 8));
        frame->distance = (uint16_t)((uint16_t)p[8] |
                                     ((uint16_t)p[9] << 8));
        frame->quality  = p[10];
        frame->version  = p[11];
        frame->XORSum   = p[12];
        frame->end      = p[13];

        return 1U;
    }

    return 0U;
}
