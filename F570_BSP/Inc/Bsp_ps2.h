#ifndef __BSP_PS2_H
#define __BSP_PS2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
/*识别插入的USB设备是有线，无线安卓，无线PC手柄*/
//有线ps2手柄设备标识符
#define Wired_PS2_VID 0x0810
#define Wired_PS2_PID 0x0001

//无线ps2手柄安卓模式设备标识符
#define Wireless_Android_PS2_VID 0x045E
#define Wireless_Android_PS2_PID 0x028E

//无线ps2手柄PC模式设备标识符
#define Wireless_PC_PS2_VID 0x0079
#define Wireless_PC_PS2_PID 0x0126

//PS2手柄类型定义
typedef enum{
	UnKnown_Dev,          //未知设备
	Wired_PS2,            //有线ps2手柄
	Wiredless_PC_PS2,     //无线ps2手柄pc模式
	Wiredless_Android_PS2,//无线ps2手柄安卓模式
}PS2_TYPE_t;

//表示按键的状态值
#define PS2KEY_PressDOWN 1  //按键按下
#define PS2KEY_PressUP   0  //按键松开

//PS2按键状态(无状态、单击、双击、长按)
typedef enum{
    PS2KEYSTATE_NONE,
    PS2KEYSTATE_SINGLECLICK,
    PS2KEYSTATE_DOUBLECLICK,
    PS2KEYSTATE_LONGCLICK
}PS2KEY_State_t;

//PS2手柄对象
typedef struct{
	uint8_t LX;      //4个方向摇杆值,取值0~255
	uint8_t LY;
	uint8_t RX;
	uint8_t RY;
	PS2KEY_State_t (*getKeyEvent)(uint8_t keybit); //获取按键事件,有单击、双击、长按3种事件可以获取,入口参数为按键的枚举键值
	uint8_t (*getKeyState)(uint8_t keybit);        //获取按键的状态,0表示按下,1表示松开,入口参数为按键的枚举键值
}PS2INFO_t;

//PS2按键位置枚举
enum 
{
	PS2KEY_SELECT	   = 0, //选择按键 （select）
	PS2KEY_LROCKER      , //左右摇杆按下键值
	PS2KEY_RROCKER      ,
	PS2KEY_START        , //开始按键（start）
	PS2KEY_UP           , //左按键区域（四个方向）
	PS2KEY_RIGHT        ,
	PS2KEY_DOWN         ,
	PS2KEY_LEFT         ,
	PS2KEY_L2           ,	//左右扳机按键值（L2,R2,L1，R1）
	PS2KEY_R2           ,
	PS2KEY_L1           ,  
	PS2KEY_R1           ,
	PS2KEY_1GREEN       , //右按键区域(四种不同颜色）
	PS2KEY_2RED         , 
	PS2KEY_3BLUE        , 
	PS2KEY_4PINK           
};

/* ============================================================================
 * 手柄接口层(非本人所写，这一块借助AI与手柄商家的建议）
 * ----------------------------------------------------------------------------
 * 风格继续沿用解耦多任务风格：上层（Control_task）或者调试层（PS2_task)只认这个
 * 接口，不直接碰 ps2_info、不关心底层到底是有线/无线PC/无线安卓哪种协议。
 *
 * 注意：这里的快照只包含 getKeyState（持续按下/松开），不包含
 * getKeyEvent（单击/双击/长按）。getKeyEvent 内部是有状态的检测状态机
 * （每个按键自己记录当前处于哪个检测阶段），设计前提是"每帧数据到达时
 * 只被调用一次"。目前它的唯一调用者是 PS2_task.c（验证层，用来测试
 * 按键功能）。如果 Control_task 以后也要用它做"点一下切换模式"这类
 * 操作，需要正式把调用权转移过来，并同时关掉 PS2_task.c 里对应的调用，
 * 不能两边同时调，否则会互相抢状态、导致单双击判断错乱。
 * ============================================================================
 */
typedef struct
{
	uint8_t  LX;         //左摇杆X，0~255，中位约127（同 ps2_info.LX）
	uint8_t  LY;
	uint8_t  RX;
	uint8_t  RY;
	uint16_t key_state;  //16个按键的原始状态位图，bit定义复用上面的 PS2KEY_xxx 枚举
	uint8_t  connected;  //1=手柄已连接并完成HID类初始化，0=未连接/已拔出
}GamepadInput_t;

/* 驱动接口 */
typedef struct
{
	void (*init)(void);
	void (*get_input)(GamepadInput_t *out);
}GamepadInterface_t, *pGamepadInterface_t;

extern GamepadInterface_t UserGamepad;   //UserGamepad包含六组结构体数据

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PS2_H */
