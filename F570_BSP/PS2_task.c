/* ============================================================================
 * PS2_task.c —— 手柄功能【验证层】//开发者初次接触手柄可以用来验证功能，后续可以删掉
 * ----------------------------------------------------------------------------
 * 分层关系（从下到上）：
 *
 *   Bsp_ps2.c        [驱动层]  解析 USB HID 原始报文
 *        │                     -> 填充 ps2_info.LX/LY/RX/RY
 *        │                     -> 填充 ps2_info.getKeyEvent()/getKeyState()
 *        ▼
 *   PS2_task.c       [验证层]  ←这里
 *                              PS2_task改成RTOS任务，使用队列进行信号传递
 *        ▼
 *   Control_task.c   [控制层]  ← 这里
 *                              读取同样的 ps2_info，把摇杆/按键翻译成
 *                              油门、姿态目标、模式切换等飞控指令。
 *
 * ============================================================================
 */
//按键的弱函数
#include "usbh_hid.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Bsp_ps2.h"
#include <stdio.h>
#include "Control_task.h"
#include "task.h"

#define DEG_TO_RAD 0.017453292519943f
/* ========== 外部资源 ========== */
extern SemaphoreHandle_t g_xMutex_Printf;                       // printf 互斥锁
extern USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost); // 驱动层解码函数
extern PS2INFO_t ps2_info;                                      // 驱动层已解析好的数据源

/* ============================================================================
 * 按键 bit 编号 —— 用命名宏代替魔法数字
 * 对照表来自 Bsp_ps2.c 里三种手柄型号统一后的 ps2_KeyVal 位图定义
 * ============================================================================
 *  bit | 宏名               | 按键
 * -----|--------------------|------------------------------
 *   0  | PS2_KEY_SELECT     | SELECT（中间左边小按钮）
 *   1  | PS2_KEY_L3         | 左摇杆按下
 *   2  | PS2_KEY_R3         | 右摇杆按下
 *   3  | PS2_KEY_START      | START（中间右边小按钮）
 *   4  | PS2_KEY_UP         | 方向键 ↑
 *   5  | PS2_KEY_RIGHT      | 方向键 →
 *   6  | PS2_KEY_DOWN       | 方向键 ↓
 *   7  | PS2_KEY_LEFT       | 方向键 ←
 *   8  | PS2_KEY_L2         | 左扳机2号（下层）
 *   9  | PS2_KEY_R2         | 右扳机2号（下层）
 *  10  | PS2_KEY_L1         | 左扳机1号（上层）
 *  11  | PS2_KEY_R1         | 右扳机1号（上层）
 *  12  | PS2_KEY_GREEN (Y)  | 右侧四色键最上面
 *  13  | PS2_KEY_RED   (B)  | 右侧四色键最右边
 *  14  | PS2_KEY_BLUE  (A)  | 右侧四色键最下面
 *  15  | PS2_KEY_PINK  (X)  | 右侧四色键最左边
 * ============================================================================
 */
#define PS2_KEY_SELECT   0
#define PS2_KEY_L3       1
#define PS2_KEY_R3       2
#define PS2_KEY_START    3
#define PS2_KEY_UP       4
#define PS2_KEY_RIGHT    5
#define PS2_KEY_DOWN     6
#define PS2_KEY_LEFT     7
#define PS2_KEY_L2       8
#define PS2_KEY_R2       9
#define PS2_KEY_L1       10
#define PS2_KEY_R1       11
#define PS2_KEY_GREEN    12   /* Y */
#define PS2_KEY_RED      13   /* B */
#define PS2_KEY_BLUE     14   /* A */
#define PS2_KEY_PINK     15   /* X */

/* ============================================================================
 * 测试开关 —— 所有开关集中在这里，改数字就能启用/关闭对应测试，
 * ============================================================================
 */
#define TEST_JOYSTICK        0  /* 摇杆原始值 LX/LY/RX/RY            */
#define TEST_FACE_BUTTONS    0   /* 四色键：单击/双击/长按事件         */
#define TEST_TRIGGERS        0   /* 扳机 L1/L2/R1/R2：持续按住状态     */
#define TEST_DPAD            0   /* 方向键：持续按住状态               */
#define TEST_SELECT_START    0   /* SELECT / START：单击事件           */
#define TEST_STICK_PRESS     0   /* 摇杆按下 L3/R3：单击事件           */

/* ============================================================================
 * 通用小工具函数
 * 目的：避免每加一个按键就复制一遍 if-else，四色键/扳机/方向键
 * 本质上都是"给个按键编号 + 名字，打印对应状态"，抽出来复用
 * ============================================================================
 */

/* 打印一个按键的边沿事件（单击/双击/长按）。适合"按一下触发一次动作"的按键 */
static void PrintKeyEvent(const char *name, uint8_t bit)
{
    PS2KEY_State_t event = ps2_info.getKeyEvent(bit);

    switch (event)
    {
        case PS2KEYSTATE_SINGLECLICK://单击
            printf("%s\r\n", name);
            break;
        case PS2KEYSTATE_DOUBLECLICK://双击
            printf("double %s\r\n", name);
            break;
        case PS2KEYSTATE_LONGCLICK://长按
            printf("long %s\r\n", name);
            break;
        default:
            /* PS2KEYSTATE_NONE，本次没有事件，不打印 */
            break;
    }
}

/* 打印一个按键的持续按住状态。适合"按住就要一直生效"的按键，比如扳机 */
static void PrintKeyHeld(const char *name, uint8_t bit)
{
    if (ps2_info.getKeyState(bit) == PS2KEY_PressDOWN)
    {
        printf("%s\r\n", name);
    }
}

#define PS2_TASK_FREQ 20U//手柄的RTOS周期要区别于control的周期，这里设置为20hz
static void PS2_SendControl(void)
{
    GamepadInput_t gamepad;//手柄数据
    FlyControlType_t cmd;//手柄控制姿态角数据

    UserGamepad.get_input(&gamepad);//获取手柄数据

    cmd.source = PS2Cmd;//认证是PS2的数据

    cmd.pitch = 0.0f;
    cmd.roll = 0.0f;
    cmd.gyroz = 0.0f;
    cmd.height = 0.0f;

    /* LY → Roll */
    if(gamepad.LY < 64)
    {
        cmd.roll = 10.0f * DEG_TO_RAD;
    }
    else if(gamepad.LY > 190)
    {
        cmd.roll = -10.0f * DEG_TO_RAD;
    }

    /* LX → Pitch */
    if(gamepad.LX < 64)
    {
        cmd.pitch = 10.0f * DEG_TO_RAD;
    }
    else if(gamepad.LX > 190)
    {
        cmd.pitch = -10.0f * DEG_TO_RAD;
    }

    /* RX → Yaw方向 */
    if(gamepad.RX < 64)
    {
        cmd.gyroz = 1.0f;
    }
    else if(gamepad.RX > 190)
    {
        cmd.gyroz = -1.0f;
    }
		
    /* RY → Height方向 */
		 if(gamepad.RY < 64)
    {
        cmd.height = 0.025f;
    }
    else if(gamepad.RY > 190)
    {
        cmd.height=-0.025f;
    }
    WriteFlyControlQueue(cmd, 0, NULL);
}
void PS2_task(void *param)//手柄RTOS任务
{
    TickType_t pretime;
    (void)param;
    pretime = xTaskGetTickCount();

    for(;;)
    {
        /* 读取手柄并发送控制指令 */
        PS2_SendControl();

        /* 20Hz，50ms执行一次 */
        vTaskDelayUntil(&pretime,
                        pdMS_TO_TICKS(1000U / PS2_TASK_FREQ));
    }
}

/* ============================================================================
 * 各测试项 —— 每个函数只负责一类按键，方便单独启用/关闭/删除
 * ============================================================================
 */

#if TEST_JOYSTICK
static void Test_Joystick(void)
{
    printf("LX=%3d, LY=%3d, RX=%3d, RY=%3d\r\n",
           ps2_info.LX, ps2_info.LY, ps2_info.RX, ps2_info.RY);
}
#endif

#if TEST_FACE_BUTTONS
static void Test_FaceButtons(void)
{
    PrintKeyEvent("Y", PS2_KEY_GREEN);
    PrintKeyEvent("B", PS2_KEY_RED);
    PrintKeyEvent("A", PS2_KEY_BLUE);
    PrintKeyEvent("X", PS2_KEY_PINK);
}
#endif

#if TEST_TRIGGERS
static void Test_Triggers(void)
{
    PrintKeyHeld("L2", PS2_KEY_L2);
    PrintKeyHeld("R2", PS2_KEY_R2);
    PrintKeyHeld("L1", PS2_KEY_L1);
    PrintKeyHeld("R1", PS2_KEY_R1);
}
#endif

#if TEST_DPAD
static void Test_Dpad(void)
{
    PrintKeyHeld("up", PS2_KEY_UP);
    PrintKeyHeld("Right", PS2_KEY_RIGHT);
    PrintKeyHeld("Down", PS2_KEY_DOWN);
    PrintKeyHeld("Left", PS2_KEY_LEFT);
}
#endif

#if TEST_SELECT_START
static void Test_SelectStart(void)
{
    PrintKeyEvent("SELECT", PS2_KEY_SELECT);
    PrintKeyEvent("START", PS2_KEY_START);
}
#endif

#if TEST_STICK_PRESS
static void Test_StickPress(void)
{
    PrintKeyEvent("Left_Press L3", PS2_KEY_L3);
    PrintKeyEvent("Right_Press R3", PS2_KEY_R3);
}
#endif

/* ============================================================================
 * USB HID 事件回调
 * ----------------------------------------------------------------------------
 * 触发时机：USB Host 中间件每收到一帧新的手柄 report 就调用一次
 *          （由内部的 USBH_HID_Process 状态机驱动，频率约等于手柄的
 *           poll interval，通常几毫秒一次）
 *
 * 本函数结构固定为两步，不要在这里加任何"决策"逻辑：
 *   第一步：调驱动层解码，把最新一帧数据写入 ps2_info
 *   第二步：只读 ps2_info，打印出来看对不对
 * ============================================================================
 */
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    /* 第一步：驱动层解码 —— 这一步之后 ps2_info 就是最新数据了 */
    USBH_HID_PS2_Decode(phost);

    /* 第二步：验证层只读、只打印，加锁避免和其他任务的 printf 交叉 */
    if (xSemaphoreTake(g_xMutex_Printf, portMAX_DELAY) == pdTRUE)
    {
#if TEST_JOYSTICK
        Test_Joystick();
#endif
#if TEST_FACE_BUTTONS
        Test_FaceButtons();
#endif
#if TEST_TRIGGERS
        Test_Triggers();
#endif
#if TEST_DPAD
        Test_Dpad();
#endif
#if TEST_SELECT_START
        Test_SelectStart();
#endif
#if TEST_STICK_PRESS
        Test_StickPress();
#endif

        xSemaphoreGive(g_xMutex_Printf);
    }
}
