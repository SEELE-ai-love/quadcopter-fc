/* LC307_task.c */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "tim.h"
#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h" 
#include "task.h"
#include "queue.h"
#include "LC307_task.h"
#include "Bsp_LC307.h"
#include "attitude_task.h"
#include "usart.h"
//注释：整个流程是DMA缓冲区
//  ↓ UART回调：写入DataLen，xQueueSendFromISR复制整个数据块（uart_callback.c）
//RTOS队列
//  ↓ LC307_task：xQueueReceive复制到queue_recv
//queue_recv.Buf中的字节
//  ↓ 状态机逐字节拼帧
//lc307_buffer[14]
//  ↓ 帧头、校验、帧尾检查
//lc307_sensor(最后的数据)
#define LC307_PACKET_LEN  14U

#define LC307_TIMEOUT_MS       100U
#define LC307_CHECK_PERIOD_MS  5U

/* 待确认工程窗口：不是已确认的 LC307 协议范围。 */
#define LC307_TIMESPAN_MIN_US  1000U
#define LC307_TIMESPAN_MAX_US  60000U

/* ARMCC5/C99 编译期布局检查：条件不满足时数组长度为负，编译失败。 */
typedef char LC307_CheckSize[(sizeof(OpticalFlowFrame_t) == LC307_PACKET_LEN) ? 1 : -1];
typedef char LC307_CheckHead[(offsetof(OpticalFlowFrame_t, head1) == 0U) ? 1 : -1];
typedef char LC307_CheckLength[(offsetof(OpticalFlowFrame_t, bufcount) == 1U) ? 1 : -1];
typedef char LC307_CheckFlowX[(offsetof(OpticalFlowFrame_t, flowX) == 2U) ? 1 : -1];
typedef char LC307_CheckFlowY[(offsetof(OpticalFlowFrame_t, flowY) == 4U) ? 1 : -1];
typedef char LC307_CheckTimespan[(offsetof(OpticalFlowFrame_t, timespan) == 6U) ? 1 : -1];
typedef char LC307_CheckDistance[(offsetof(OpticalFlowFrame_t, distance) == 8U) ? 1 : -1];
typedef char LC307_CheckQuality[(offsetof(OpticalFlowFrame_t, quality) == 10U) ? 1 : -1];
typedef char LC307_CheckVersion[(offsetof(OpticalFlowFrame_t, version) == 11U) ? 1 : -1];
typedef char LC307_CheckChecksum[(offsetof(OpticalFlowFrame_t, XORSum) == 12U) ? 1 : -1];
typedef char LC307_CheckEnd[(offsetof(OpticalFlowFrame_t, end) == 13U) ? 1 : -1];

/*============================ 
        光流数据
=============================*/ 
static FlyPosMsg_t pos = {0};

static uint8_t lc307_valid = 0U;
static TickType_t last_good_tick = 0U;
static uint32_t lc307_sequence = 0U;
static uint32_t lc307_fault_epoch = 0U;
static uint32_t lc307_published_rx_epoch = 0U;

//接收任务只发布观测，Control任务只发布估计，各自只有一个写入者
typedef struct
{
    OpticalFlowFrame_t frame;
    float gx,gy,gz;
} LC307_Observation_t;
static LC307_Observation_t lc307_observation = {0};
static uint8_t lc307_estimate_valid = 0U;
static TickType_t estimate_tick = 0U;
static TickType_t estimate_receive_tick = 0U;
static uint32_t estimate_sequence = 0U;
static uint32_t estimate_fault_epoch = 0U;
static uint32_t estimate_rx_epoch = 0U;
#define LC307_IMU_MAX_GAP_MS 25U//待验证工程窗口，不能跨长时间补积分
static volatile uint32_t lc307_prediction_count = 0U;//只统计成功发布的新IMU预测
static volatile uint32_t lc307_correction_count = 0U;//只统计成功发布的新光流校正
// IMU_FIFO：本计数现在也包含sequence不连续，overflow另外看FIFO统计
static volatile uint32_t lc307_estimator_gap_count = 0U;//只统计超过窗口的IMU间隔
static volatile uint32_t lc307_estimator_call_count = 0U;//先看估计函数有没有进来
static volatile uint8_t lc307_estimator_state = 0U;//0未进入，1进入，2输入无效，3等新帧，4间隔大，5数值异常，6发布被撤销，7成功

/* 以下诊断计数仅由 LC307_task 写入。 */
static uint32_t checksum_fail = 0U;
static uint32_t end_marker_fail = 0U;
static uint32_t timespan_fail = 0U;
static uint32_t task_invalid_len = 0U;
static uint32_t valid_frame_count = 0U;
static uint32_t timeout_count = 0U;
static uint32_t header_length_fail = 0U;

/* 在 timespan 范围过滤之前，统计通过帧结构和 XOR 的候选帧。 */
static uint32_t envelope_frame_count = 0U;
static uint16_t timespan_observed_min = UINT16_MAX;
static uint16_t timespan_observed_max = 0U;

static float x_speed_prev = 0.0f;//上一次的x轴速度
static float y_speed_prev = 0.0f;//上一次的y轴速度

//typedef struct
//{
//    uint16_t DataLen;
//    uint8_t  Buf[userconfig_LC307_DMABUF_LEN];
//} DMABuf_oridata_LC307_t;
static DMABuf_oridata_LC307_t queue_recv;// task从队列接收一个 DMA 数据块的临时变量。它的类型必须和DMA，回调类型一致
static uint8_t lc307_buffer[LC307_PACKET_LEN];
static uint8_t bufferCnt = 0U;//传输到第几位了
static uint8_t state_m = 0U;//判断帧头是否识别成功
static uint8_t LastRecv = 0U;
static OpticalFlowFrame_t lc307_sensor;//数据存放的地方
extern SemaphoreHandle_t g_xMutex_Printf; //互斥锁

static uint8_t XOR_Checksum(const uint8_t *data, uint16_t length)
{
    uint8_t result = 0U;

    for (uint16_t i = 0U; i < length; i++)
    {
        result ^= data[i];
    }

    return result;
}

static uint8_t LC307_CheckEnvelope(void)
{
    uint8_t accepted = 1U;

    if ((lc307_buffer[0] != 0xFEU) ||
        (lc307_buffer[1] != 0x0AU))
    {
        header_length_fail++;
        accepted = 0U;
    }

    if (lc307_buffer[12] !=
        XOR_Checksum(&lc307_buffer[2], 10U))
    {
        checksum_fail++;
        accepted = 0U;
    }

    if (lc307_buffer[13] != 0x55U)
    {
        end_marker_fail++;
        accepted = 0U;
    }

    return accepted;
}

static void LC307_Invalidate(void)
{
    taskENTER_CRITICAL();
    lc307_valid = 0U;
    lc307_fault_epoch++;
    taskEXIT_CRITICAL();
    //这里只撤销有效状态，不清位置、速度和上一次速度
}

uint8_t LC307_GetLatestEx(FlyPosMsg_t *out,LC307_Status_t *status)
{
    uint8_t valid;
    TickType_t good_tick;
    TickType_t predicted_tick;
    TickType_t now;

    if(out == NULL || status == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = pos;
    valid = (uint8_t)(lc307_valid != 0U && lc307_estimate_valid != 0U);
    good_tick = estimate_receive_tick;
    predicted_tick = estimate_tick;
    status->sequence = estimate_sequence;
    status->fault_epoch = lc307_fault_epoch;
    status->rx_epoch = g_lc307_rx_epoch;
    if(g_lc307_config_ok == 0U || estimate_rx_epoch != status->rx_epoch || estimate_fault_epoch != lc307_fault_epoch)
    {
        valid = 0U;
    }
    taskEXIT_CRITICAL();

    now = xTaskGetTickCount();
    status->age_ms = (uint32_t)((uint64_t)(TickType_t)(now-good_tick)*1000U/configTICK_RATE_HZ);
    //当前1个Tick就是1ms，同一DMA块的多帧保留相同接收时间，不伪造时间戳
    status->receive_ms = (uint32_t)good_tick;
    if(status->age_ms > LC307_TIMEOUT_MS ||
       (TickType_t)(now-predicted_tick) > pdMS_TO_TICKS(LC307_IMU_MAX_GAP_MS))
    {
        valid = 0U;
    }
    if(valid == 0U)
    {
        memset(out,0,sizeof(*out));
    }
    return valid;
}

/*关于DMA所有权：运行期间只有LC307任务可以启动和中止USART3接收*/
static HAL_StatusTypeDef LC307_StartReceive(void)
{
    HAL_StatusTypeDef result;
    if(g_lc307_config_ok == 0U || huart3.hdmarx == NULL ||
       huart3.hdmarx->Init.Mode != DMA_NORMAL)
    {
        return HAL_ERROR;
    }
    if(huart3.RxState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    result = HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DMABuf_oridata_LC307.Buf,
                                         userconfig_LC307_DMABUF_LEN);
    if(result == HAL_OK)
    {
        //HAL每次启动可能重新开启HT，这里只处理完整数据块，所以每次关闭HT
        __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
    }
    return result;
}

static void LC307_ServiceRx(void)
{
    static TickType_t last_recover_attempt = 0U;
    TickType_t now;
    uint8_t requests;
    uint8_t recover_due;
    uint8_t rx_ready;
    HAL_StatusTypeDef result;

    if(g_lc307_config_ok == 0U || huart3.hdmarx == NULL ||
       huart3.hdmarx->Init.Mode != DMA_NORMAL)
    {
        return;
    }

    now = xTaskGetTickCount();
    recover_due = (uint8_t)((TickType_t)(now-last_recover_attempt) >= pdMS_TO_TICKS(100U));
    rx_ready = (uint8_t)(huart3.RxState == HAL_UART_STATE_READY);

    //短临界区只取出请求，HAL运行期间中断新提出的请求留到下一轮
    taskENTER_CRITICAL();
    requests = g_lc307_rx_requests;
    if((requests & LC307_RX_REQUEST_RECOVER) != 0U &&
       (recover_due == 0U || rx_ready == 0U))
    {
        taskEXIT_CRITICAL();
        return;
    }
    g_lc307_rx_requests = 0U;
    if((requests & LC307_RX_REQUEST_RECOVER) != 0U)
    {
        g_lc307_rx_epoch++;
    }
    taskEXIT_CRITICAL();

    if(requests == 0U)
    {
        return;
    }
    if((requests & LC307_RX_REQUEST_RECOVER) != 0U)
    {
        last_recover_attempt = now;
        LC307_Invalidate();
        //HAL接收流程结束进入READY之后才清理，HAL操作不放进临界区
        result = HAL_UART_AbortReceive(&huart3);
        if(result != HAL_OK)
        {
            taskENTER_CRITICAL();
            g_lc307_rx_requests |= LC307_RX_REQUEST_RECOVER;
            taskEXIT_CRITICAL();
            return;
        }
    }
    else if(huart3.RxState != HAL_UART_STATE_READY)
    {
        //普通启动请求已经过时，当前接收仍在运行，不重复启动也不中止
        return;
    }

    result = LC307_StartReceive();
    if(result != HAL_OK)
    {
        LC307_Invalidate();
        taskENTER_CRITICAL();
        g_lc307_rx_epoch++;
        g_lc307_rx_requests |= LC307_RX_REQUEST_RECOVER;
        taskEXIT_CRITICAL();
    }
}

static uint8_t LC307_Publish(const OpticalFlowFrame_t *next,TickType_t receive_tick,uint32_t receive_epoch)
{
    TickType_t now;
    AttitudeSnapshot_t imu;
    uint8_t published = 0U;

    if(Attitude_GetSnapshot(&imu) == 0U || !isfinite(imu.gx) ||
       !isfinite(imu.gy) || !isfinite(imu.gz))
    {
        return 0U;
    }
    now = xTaskGetTickCount();
    if((TickType_t)(now-receive_tick) > pdMS_TO_TICKS(LC307_TIMEOUT_MS) ||
       (TickType_t)(now-imu.sample_tick) > pdMS_TO_TICKS(LC307_IMU_MAX_GAP_MS))
    {
        LC307_Invalidate();
        return 0U;
    }

    //这里只发布结果，预测、校正、积分都已经在临界区外计算完成
    //补充：本轮只发布完整帧和旋转补偿用的gyro，200Hz预测另有唯一写入者
    taskENTER_CRITICAL();
    if(receive_epoch == g_lc307_rx_epoch && g_lc307_config_ok != 0U)
    {
        lc307_observation.frame = *next;
        lc307_observation.gx = imu.gx;
        lc307_observation.gy = imu.gy;
        lc307_observation.gz = imu.gz;
        last_good_tick = receive_tick;
        lc307_published_rx_epoch = receive_epoch;
        lc307_sequence++;
        lc307_valid = 1U;
        published = 1U;
    }
    taskEXIT_CRITICAL();
    return published;
}

static void LC307_EstimatorPause(void)
{
    //只撤销估计有效状态，原位置、速度和previous speed保留
    taskENTER_CRITICAL();
    lc307_estimate_valid = 0U;
    taskEXIT_CRITICAL();
}

// IMU_FIFO BEGIN：每次处理一个带原采样时间的快照，仍然只有Control调用
static void LC307_EstimatorUpdateSample(const AttitudeSnapshot_t *sample,uint8_t allow_correction)
{
    static uint8_t tracking = 0U;
    static TickType_t previous_imu_tick = 0U;
    static uint32_t previous_imu_sequence = 0U;
    LC307_Observation_t observation;
    OpticalFlowFrame_t lc307_sensor;
    AttitudeSnapshot_t imu;
    FlyPosMsg_t base_pos;
    TickType_t now,receive_tick;
    uint32_t sequence,receive_epoch,fault_epoch;
    uint8_t valid,new_frame,published = 0U;
    float imu_dt = 0.0f;
    float flow_dt;//dt时间
    float flow_rate_x, flow_rate_y;//x、y轴光流角速度
    float height_m, gyro_x, gyro_y;//高度m/角速度gx/角速度gy，注意这里是弧度制，与姿态解算的单位制一样
    float flow_vx, flow_vy;//光流测得的x、y轴速度
    float ax, ay;//x、y轴加速度，加速度*时间=速度变化量
    float ax_before_deadzone,ay_before_deadzone;//保留死区前的去重力加速度
    float vx_predict, vy_predict;//预测速度
    FlyPosMsg_t next_pos;
    float previous_x_speed, previous_y_speed;
    TickType_t publish_tick;
	  //姿态数据
    float roll, pitch, yaw;
    float gx, gy, gz;
    float acc_x, acc_y, acc_z;
    float gravity_x, gravity_y, gravity_z;//光流计融合后的数据
    const float gravity = 9.8f;//重力加速度

    //只有Control任务调用本函数，接收任务不再对pos进行积分
    lc307_estimator_state = 1U;
    taskENTER_CRITICAL();
    observation = lc307_observation;
    base_pos = pos;
    receive_tick = last_good_tick;
    sequence = lc307_sequence;
    receive_epoch = lc307_published_rx_epoch;
    fault_epoch = lc307_fault_epoch;
    valid = (uint8_t)(lc307_valid != 0U && g_lc307_config_ok != 0U &&
                      receive_epoch == g_lc307_rx_epoch);
    //一批IMU先逐帧预测，只在最后一帧使用最新光流校正一次
    new_frame = (uint8_t)(allow_correction != 0U && sequence != estimate_sequence);
    if(estimate_rx_epoch != receive_epoch || estimate_fault_epoch != fault_epoch)
    {
        tracking = 0U;
    }
    taskEXIT_CRITICAL();

    imu = *sample;
    valid = (uint8_t)(imu.sequence != 0U && valid != 0U);
    now = xTaskGetTickCount();
    if(valid == 0U ||
       (TickType_t)(now-receive_tick) > pdMS_TO_TICKS(LC307_TIMEOUT_MS) ||
       (TickType_t)(now-imu.sample_tick) > pdMS_TO_TICKS(LC307_IMU_MAX_GAP_MS) ||
       !isfinite(imu.roll) || !isfinite(imu.pitch) ||
       !isfinite(imu.ax) || !isfinite(imu.ay))
    {
        lc307_estimator_state = 2U;
        LC307_EstimatorPause();
        tracking = 0U;
        return;
    }
    if(tracking == 0U && new_frame == 0U)
    {
        lc307_estimator_state = 3U;
        LC307_EstimatorPause();
        return;
    }
    if(tracking != 0U && imu.sequence != previous_imu_sequence)
    {
        TickType_t imu_ticks = (TickType_t)(imu.sample_tick-previous_imu_tick);
        //FIFO溢出或中间缺帧都不跨过去补积分，保留原来位置和速度，等待新观测
        if(imu.sequence != (uint32_t)(previous_imu_sequence+1U) ||
           imu_ticks > pdMS_TO_TICKS(LC307_IMU_MAX_GAP_MS))
        {
            lc307_estimator_gap_count++;
            lc307_estimator_state = 4U;
            LC307_EstimatorPause();
            tracking = 0U;
            return;
        }
        imu_dt = (float)imu_ticks/(float)configTICK_RATE_HZ;
    }
    //同一IMU样本不重复预测，同一DMA时间组不伪造sample_dt
    lc307_sensor = observation.frame;
    /* 第1步：积分时间 us -> s */
    flow_dt = (float)lc307_sensor.timespan * 1.0e-6f;//乘以10的-6次方得到秒

    /* 第2步：（原始数据/10000）/时间=角速度
     积分光流 -> 光流角速度 rad/s。
       x 对应 -flowY，y 对应 -flowX */
    /* 上方旧注释保留；当前按已验证映射：x=flowX，y=-flowY。 */
    flow_rate_x = (float)lc307_sensor.flowX / 10000.0f / flow_dt;
    flow_rate_y = -(float)lc307_sensor.flowY / 10000.0f / flow_dt;

    /* 第3步：进入姿态数据，IMU融合*/
//    height_m = 0.50f;   /* 离地高度，m */
		 height_m = 0.0050f;   /* 离地高度（这里是静止放在桌面上的，用于不引入高度环的桌面测试阶段），m */
    roll = imu.roll; pitch = imu.pitch; yaw = imu.yaw;//获取姿态角
    gx = observation.gx; gy = observation.gy; gz = observation.gz;//获取角速度
    acc_x = imu.ax; acc_y = imu.ay; acc_z = imu.az;//获取加速度

    
    gravity_x =  gravity * sinf(pitch);
    gravity_y =  -gravity * sinf(roll) * cosf(pitch);
    gravity_z =  gravity * cosf(roll) * cosf(pitch);

    (void)gravity_z;
    
    gyro_x = gx;
    gyro_y = gy;
    ax = acc_x - gravity_x;
    ay = acc_y - gravity_y;
    ax_before_deadzone = ax;
    ay_before_deadzone = ay;

    if (fabsf(ax) < 0.15f) ax = 0.0f;
    if (fabsf(ay) < 0.15f) ay = 0.0f;

    /* 第4步：（原始数据/10000）/时间*高度=速度 与旋转补偿（减去同方向的gyro）*/
    flow_vx = height_m * (flow_rate_x + gyro_x);
    flow_vy = height_m * (flow_rate_y + gyro_y);

    /* 第5步：IMU 预测，光流校正（互补融合）
   本质： 预测速度 = 上次速度 + 加速度 × 时间
*/
    /* 本任务唯一写入 pos；在临界区外复制并完成全部计算。 */
    next_pos = base_pos;
    previous_x_speed = x_speed_prev;
    previous_y_speed = y_speed_prev;

    vx_predict = next_pos.x_speed + ax * imu_dt;
    vy_predict = next_pos.y_speed + ay * imu_dt;
		
    /* 给速度用一阶低通互补滤波，对于姿态解算加速度以及高度环高度速度一样 */
    //同一帧只校正一次，没有新帧就只做IMU预测
    if(new_frame != 0U)
    {
        next_pos.x_speed = 0.95f * vx_predict + 0.05f * flow_vx;
        next_pos.y_speed = 0.95f * vy_predict + 0.05f * flow_vy;
    }
    else
    {
        next_pos.x_speed = vx_predict;
        next_pos.y_speed = vy_predict;
    }

    /* 第6步：梯形积分，得到相对位置 */
    next_pos.x_pos += 0.5f * (next_pos.x_speed + previous_x_speed) * imu_dt;
    next_pos.y_pos += 0.5f * (next_pos.y_speed + previous_y_speed) * imu_dt;
    /* 第7步：更新数据 */
    publish_tick = now;

    /* 短临界区只发布四个 float、时间戳和有效标志。 */
    //发布函数一并保存接收代数和新帧序号，原来的融合公式保持不变
    //上方发布说明保留，这里改为发布200Hz估计，接收时间仍用真实DMA时间
    if(!isfinite(next_pos.x_pos) || !isfinite(next_pos.y_pos) ||
       !isfinite(next_pos.x_speed) || !isfinite(next_pos.y_speed))
    {
        lc307_estimator_state = 5U;
        LC307_EstimatorPause();
        tracking = 0U;
        return;
    }
    taskENTER_CRITICAL();
    if(lc307_valid != 0U && g_lc307_config_ok != 0U &&
       receive_epoch == g_lc307_rx_epoch && fault_epoch == lc307_fault_epoch)
    {
        pos = next_pos;
        estimate_receive_tick = receive_tick;
        //预测不能提前把尚未校正的光流帧标记成已经使用
        if(new_frame != 0U)
        {
            estimate_sequence = sequence;
        }
        estimate_fault_epoch = fault_epoch;
        estimate_rx_epoch = receive_epoch;
        estimate_tick = imu.sample_tick;
        lc307_estimate_valid = 1U;
        published = 1U;
    }
    taskEXIT_CRITICAL();
    if(published == 0U)
    {
        lc307_estimator_state = 6U;
        LC307_EstimatorPause();
        tracking = 0U;
        return;
    }
    (void)publish_tick;
    (void)yaw;
    (void)gz;
    (void)acc_z;

    x_speed_prev = next_pos.x_speed;
    y_speed_prev = next_pos.y_speed;
    previous_imu_tick = imu.sample_tick;
    previous_imu_sequence = imu.sequence;
    if(imu_dt > 0.0f) lc307_prediction_count++;
    if(new_frame != 0U) lc307_correction_count++;
    tracking = 1U;
    lc307_estimator_state = 7U;
//串口日志：打印出融合位移，旋转补偿融合速度，加速度，角速度是否静止维持在0左右
    {
        static uint16_t print_div = 0U;
        print_div++;
        if(print_div >= 40U)   // 200Hz / 40 = 5Hz
        {
            print_div = 0U;
            if (xSemaphoreTake(g_xMutex_Printf, 0U) == pdTRUE)
            {
//                printf("raw=%d,%d rate=%ld,%ld flow=%ld,%ld "
//                       "roll=%ld pitch=%ld gx=%ld gy=%ld ax_pre=%ld ay_pre=%ld "
//                       "fused=%ld,%ld pos=%ld,%ld\r\n",
//                       (int)lc307_sensor.flowX,//原始位移
//                       (int)lc307_sensor.flowY,
//                       (long)(flow_rate_x * 1000.0f),//原始光流角速度
//                       (long)(flow_rate_y * 1000.0f),
//                       (long)(flow_vx * 1000.0f),//原始线速度
//                       (long)(flow_vy * 1000.0f),
//                       (long)(roll * 1000.0f),//姿态角
//                       (long)(pitch * 1000.0f),
//                       (long)(gx * 1000.0f),//gyro
//                       (long)(gy * 1000.0f),
//                       (long)(ax_before_deadzone * 1000.0f),//死区之前的去重力加速度
//                       (long)(ay_before_deadzone * 1000.0f),
//                       (long)(next_pos.x_speed * 1000.0f),//融合光流速度
//                       (long)(next_pos.y_speed * 1000.0f),
//                       (long)(next_pos.x_pos * 1000.0f),//融合光流位移
//                       (long)(next_pos.y_pos * 1000.0f));
							
							
                xSemaphoreGive(g_xMutex_Printf);
            }
        }
    }
}

// IMU_FIFO END

void LC307_EstimatorUpdate(void)
{
    //FIFO空时只使用已经取出过的末帧，不能绕过FIFO去抢读刚发布的新样本
    static AttitudeSnapshot_t last_imu = {0};
    AttitudeSnapshot_t imu;
    AttitudeFifoStats_t fifo;
    uint32_t remaining;

    lc307_estimator_call_count++;
    Attitude_FifoGetStats(&fifo);
    remaining = fifo.pending;

    // IMU_FIFO BEGIN：这一轮最多取16帧，处理中刚到的新帧留到下一轮，不能无限追队列
    if(remaining == 0U)
    {
        //没有新IMU时仍可校正新光流；同一sequence不会再次预测，也不伪造dt
        if(last_imu.sequence != 0U)
        {
            LC307_EstimatorUpdateSample(&last_imu,1U);
        }
        else
        {
            lc307_estimator_state = 2U;
            LC307_EstimatorPause();
        }
        return;
    }

    while(remaining != 0U)
    {
        if(Attitude_FifoPop(&imu) == 0U)
        {
            break;
        }
        remaining--;
        last_imu = imu;
        LC307_EstimatorUpdateSample(&imu,(uint8_t)(remaining == 0U));
    }
    // IMU_FIFO END
}

static void LC307_CheckTimeout(TickType_t now)
{
    /* 本任务是状态的唯一写入者；getter 只读。 */
    if ((lc307_valid != 0U) &&
        ((TickType_t)(now - last_good_tick) >
         pdMS_TO_TICKS(LC307_TIMEOUT_MS)))
    {
        LC307_Invalidate();

        timeout_count++;
    }
}

uint8_t LC307_GetLatest(FlyPosMsg_t *out)
{
    uint8_t valid;
    TickType_t good_tick;
    TickType_t predicted_tick;
    TickType_t now;

    if (out == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = pos;
    valid = (uint8_t)(lc307_valid != 0U && lc307_estimate_valid != 0U);
    good_tick = estimate_receive_tick;
    predicted_tick = estimate_tick;
    if(g_lc307_config_ok == 0U || estimate_rx_epoch != g_lc307_rx_epoch || estimate_fault_epoch != lc307_fault_epoch)
    {
        valid = 0U;
    }
    taskEXIT_CRITICAL();

    /* 双保险：任务暂未调度时，调用方也不能使用超时数据。 */
    now = xTaskGetTickCount();

    if ((valid != 0U) &&
        ((TickType_t)(now - good_tick) >
         pdMS_TO_TICKS(LC307_TIMEOUT_MS) ||
         (TickType_t)(now-predicted_tick) > pdMS_TO_TICKS(LC307_IMU_MAX_GAP_MS)))
    {
        valid = 0U;
    }

    if (valid == 0U)
    {
        memset(out, 0, sizeof(*out));
    }

    return valid;
}

static void LC307_PrintDiagnostics(TickType_t now)
{
    static TickType_t last_print_tick = 0U;
    uint8_t valid;
    uint16_t timespan;
    uint16_t dt_min;
    uint16_t dt_max;
    TickType_t good_tick;
    uint32_t overflow;
    uint32_t age_ms;
    uint32_t prediction_count,correction_count,gap_count;
    uint32_t call_count,rx_sequence,rx_epoch,published_epoch;
    uint8_t estimator_state,config_ok,imu_ok;
    uint32_t imu_age_ms;
    AttitudeSnapshot_t imu;

    if ((TickType_t)(now - last_print_tick) <
        pdMS_TO_TICKS(1000U))
    {
        return;
    }

    last_print_tick = now;

    taskENTER_CRITICAL();
    valid = lc307_valid;
    good_tick = last_good_tick;
    timespan = lc307_sensor.timespan;
    overflow = g_lc307_queue_overflow_count;
    prediction_count = lc307_prediction_count;
    correction_count = lc307_correction_count;
    gap_count = lc307_estimator_gap_count;
    call_count = lc307_estimator_call_count;
    estimator_state = lc307_estimator_state;
    config_ok = g_lc307_config_ok;
    rx_sequence = lc307_sequence;
    rx_epoch = g_lc307_rx_epoch;
    published_epoch = lc307_published_rx_epoch;
    taskEXIT_CRITICAL();

    //这里只诊断，不改有效标志，也不修改估计结果
    imu_ok = Attitude_GetSnapshot(&imu);
    imu_age_ms = (uint32_t)((uint64_t)(TickType_t)(xTaskGetTickCount()-imu.sample_tick)*1000U/configTICK_RATE_HZ);

    /* timespan 仅供诊断；min/max 和其余计数仅由本任务读写。 */
    dt_min = (envelope_frame_count == 0U) ?
             0U : timespan_observed_min;
    dt_max = (envelope_frame_count == 0U) ?
             0U : timespan_observed_max;

    age_ms = (valid_frame_count == 0U) ? 0U :
        (uint32_t)((uint64_t)(TickType_t)(now - good_tick) *
                   1000U / configTICK_RATE_HZ);

    if (xSemaphoreTake(g_xMutex_Printf, 0U) == pdTRUE)
    {
    //    printf("LC307 valid=%u age_ms=%lu dt_us=%u "
    //           "dt_min=%u dt_max=%u envelope_frames=%lu "
    //           "checksum_fail=%lu end_marker_fail=%lu "
    //           "timespan_fail=%lu task_invalid_len=%lu "
    //           "isr_queue_full=%lu valid_frame_count=%lu "
    //           "timeout_count=%lu header_length_fail=%lu\r\n",
    //           (unsigned int)valid,
    //           (unsigned long)age_ms,
    //           (unsigned int)timespan,
    //           (unsigned int)dt_min,
    //           (unsigned int)dt_max,
    //           (unsigned long)envelope_frame_count,
    //           (unsigned long)checksum_fail,
    //           (unsigned long)end_marker_fail,
    //           (unsigned long)timespan_fail,
    //           (unsigned long)task_invalid_len,
    //           (unsigned long)overflow,
    //           (unsigned long)valid_frame_count,
    //           (unsigned long)timeout_count,
    //           (unsigned long)header_length_fail);

        // IMU_FIFO BEGIN：需要观察FIFO时再打开，沿用这里每秒一次的打印
        // AttitudeFifoStats_t fifo;
        // Attitude_FifoGetStats(&fifo);
        // printf("IMU FIFO push=%lu pop=%lu pending=%lu high=%lu overflow=%lu\r\n",
        //        (unsigned long)fifo.pushed,(unsigned long)fifo.popped,
        //        (unsigned long)fifo.pending,(unsigned long)fifo.high_water,
        //        (unsigned long)fifo.overflow);
        // IMU_FIFO END

        //第二步看预测和校正次数，按两次打印的实际时间算频率（控制层200hz，光流位置层50hz）
//        printf("LC307 RATE time_ms=%lu predict=%lu correct=%lu imu_gap=%lu\r\n",
//               (unsigned long)((uint64_t)now*1000U/configTICK_RATE_HZ),
//               (unsigned long)prediction_count,
//               (unsigned long)correction_count,
//               (unsigned long)gap_count);
    //    printf("LC307 PATH calls=%lu state=%u cfg=%u rx_valid=%u rx_seq=%lu "
    //           "imu_ok=%u imu_seq=%lu imu_age_ms=%lu rx_age_ms=%lu "
    //           "pub_epoch=%lu rx_epoch=%lu envelope=%lu good=%lu\r\n",
    //           (unsigned long)call_count,
    //           (unsigned int)estimator_state,
    //           (unsigned int)config_ok,
    //           (unsigned int)valid,
    //           (unsigned long)rx_sequence,
    //           (unsigned int)imu_ok,
    //           (unsigned long)imu.sequence,
    //           (unsigned long)imu_age_ms,
    //           (unsigned long)age_ms,
    //           (unsigned long)published_epoch,
    //           (unsigned long)rx_epoch,
    //           (unsigned long)envelope_frame_count,
    //           (unsigned long)valid_frame_count);

        xSemaphoreGive(g_xMutex_Printf);
    }
}

//整个数据包的格式是：【FE 0A 10字节数据 XOR 55】
/*============================ 
 *关于光流计传感器的整个流程：
  积分光流 → 角速度 → 旋转补偿与高度缩放
                           ↓
                与IMU预测速度互补融合
                           ↓
                  积分得到相对位置
=============================*/ 
void LC307_task(void *param)
{
    extern QueueHandle_t g_xQueueLC307_Ori;
    (void)param;
  static uint32_t frame_count = 0U;

  static uint32_t rx_count = 0U;
  static uint32_t seen_rx_epoch = 0U;
  static TickType_t previous_chunk_tick = 0U;
    for (;;)
    {
        BaseType_t received;
        TickType_t now;
        uint32_t current_rx_epoch;

        received = xQueueReceive(g_xQueueLC307_Ori,
                          &queue_recv,
                          pdMS_TO_TICKS(LC307_CHECK_PERIOD_MS));

        LC307_ServiceRx();
        now = xTaskGetTickCount();
        LC307_CheckTimeout(now);

        taskENTER_CRITICAL();
        current_rx_epoch = g_lc307_rx_epoch;
        taskEXIT_CRITICAL();
        if(current_rx_epoch != seen_rx_epoch)
        {
            seen_rx_epoch = current_rx_epoch;
            state_m = 0U;
            bufferCnt = 0U;
            LC307_Invalidate();
        }
        LC307_PrintDiagnostics(now);

        if (received != pdPASS)//判断接收队列是否成功
        {
            continue;//否，跳过
        }
        rx_count++;
        if ((rx_count == 1U) || ((rx_count % 10U) == 0U))
        {
            if (xSemaphoreTake(g_xMutex_Printf, 0U) == pdTRUE)
            {
//                printf("LC307 RX len=%u b0=%02X b1=%02X b2=%02X b3=%02X state=%u cnt=%u\r\n",
//                       (unsigned int)queue_recv.DataLen,
//                       (unsigned int)queue_recv.Buf[0],
//                       (unsigned int)queue_recv.Buf[1],
//                       (unsigned int)queue_recv.Buf[2],
//                       (unsigned int)queue_recv.Buf[3],
//                       (unsigned int)state_m,
//                       (unsigned int)bufferCnt);
							
                xSemaphoreGive(g_xMutex_Printf);
            }
        }
        if (queue_recv.DataLen > sizeof(queue_recv.Buf))//超出应该接收到的数据长度
        {
            task_invalid_len++;
            state_m = 0U;
            bufferCnt = 0U;
            LastRecv = 0U;
            continue;//跳过
        }

        //旧数据块和故障之前的数据不能重新刷新有效状态
        now = xTaskGetTickCount();
        if(queue_recv.RxEpoch != current_rx_epoch ||
           (TickType_t)(now-queue_recv.RxTick) > pdMS_TO_TICKS(LC307_TIMEOUT_MS))
        {
            state_m = 0U;
            bufferCnt = 0U;
            LC307_Invalidate();
            continue;
        }
        //长时间前留下的半帧不能和新数据继续拼接
        if(state_m != 0U &&
           (TickType_t)(queue_recv.RxTick-previous_chunk_tick) > pdMS_TO_TICKS(LC307_TIMEOUT_MS))
        {
            state_m = 0U;
            bufferCnt = 0U;
        }
        previous_chunk_tick = queue_recv.RxTick;

        for (uint16_t i = 0U; i < queue_recv.DataLen; i++)
        {
            uint8_t byte = queue_recv.Buf[i];

            if (state_m == 0U)
            {
                /* 寻找帧头 FE 0A */
                if (byte == 0xFEU)
                {
                    lc307_buffer[0] = 0xFEU;
                    bufferCnt = 1U;
                    state_m = 1U;
                }
            }
            else
            {
                lc307_buffer[bufferCnt++] = byte;

                if (bufferCnt == LC307_PACKET_LEN)
                {
                    /* 校验字节：第2至第11下标异或；帧尾为55 */
                    if (LC307_CheckEnvelope() != 0U)
                    {
                        /*
                         * 仅当结构体大小确实为14字节时复制。
                         * 该结构体还必须与协议字段布局一致。
                         */

                        if (sizeof(lc307_sensor) == LC307_PACKET_LEN)
                        {
  //关于memcpy数据融合（目标内存地址，原内存地址，拷贝多少字节），将数据进行拷贝，拷贝进lc307_sensor
                            memcpy(&lc307_sensor,
                                   lc307_buffer,
                                   LC307_PACKET_LEN);

/* ===== 光流数据处理：数据处理和融合从这里开始 ===== */
frame_count++;

envelope_frame_count++;
if (lc307_sensor.timespan < timespan_observed_min)
{
    timespan_observed_min = lc307_sensor.timespan;
}
if (lc307_sensor.timespan > timespan_observed_max)
{
    timespan_observed_max = lc307_sensor.timespan;
}

/*============================ 
 *关于数据计算：
1、（原始数据/10000）*高度=位移
2、（原始数据/10000）*高度/时间=速度
  3，（原始数据/10000）/时间=角速度
=============================*/ 
if ((lc307_sensor.timespan >= LC307_TIMESPAN_MIN_US) &&
    (lc307_sensor.timespan <= LC307_TIMESPAN_MAX_US))
{
    FlyPosMsg_t next_pos;
    if(LC307_Publish(&lc307_sensor,queue_recv.RxTick,queue_recv.RxEpoch) == 0U)
    {
        state_m = 0U;
        bufferCnt = 0U;
        continue;
    }
    //串口打印读取估计副本，不参与控制，也不改变位置和速度
    taskENTER_CRITICAL();
    next_pos = pos;
    taskEXIT_CRITICAL();

    valid_frame_count++;

    if ((frame_count % 30U) == 0U)
    {
        if (xSemaphoreTake(g_xMutex_Printf, 0U) == pdTRUE)
        {

//串口日志：打印出x，y的速度以及IMU融合之后的速度，静止时候都应该是0，向着正方向运动都应该增加并且两者同向
//       printf("flow=%ld,%ld fused=%ld,%ld\r\n",
//       (long)(flow_vx * 1000.0f),       // 光流速度 X、Y，mm/s
//       (long)(flow_vy * 1000.0f),
//       (long)(pos.x_speed * 1000.0f),   // 融合速度 X、Y，mm/s
//       (long)(pos.y_speed * 1000.0f));
//串口日志：打印出位移和融合速度，静止时候为0，方向相同
//       printf("fused=%ld,%ld pos=%ld,%ld\r\n",
//       (long)(next_pos.x_speed * 1000.0f),
//       (long)(next_pos.y_speed * 1000.0f),
//       (long)(next_pos.x_pos * 1000.0f),
//       (long)(next_pos.y_pos * 1000.0f));
            xSemaphoreGive(g_xMutex_Printf);
        }
    }
}
else
{
    timespan_fail++;
}
                        }
                    }
                    /* 无论校验是否通过，收满一帧后重新找帧头 */
                    state_m = 0U;
                    bufferCnt = 0U;
                }
            }

            LastRecv = byte;
        }
    }
}
