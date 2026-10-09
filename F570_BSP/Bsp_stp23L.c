#include "Bsp_stp23L.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"




OriData_STP23L_t DMABuf_oridata_stp23L;
float g_readonly_distance =0;
// ============================================================
//关于：#pragma pack（1）
//        结构体
//      #pragma pack ( )
//两者同时出现，作用于结构体的两端，用于防止结构体内存对齐，比如结构体最大的字节是4字节，那么系统会把所有的字节都对对齐到4字节，会出现填充空隙
// ============================================================
#pragma pack(1)
typedef struct 
{
	int16_t distance; //距离
	uint16_t noise;   //噪声强度
	uint32_t peak;    //强度
	uint8_t confidence;  //置信度
	uint32_t intg;   //积分次数
	int16_t reftof;  //温度补偿值
}LidarPointTypedef;
#pragma pack()


#pragma pack(1)
typedef struct{
    LidarPointTypedef PointCloud[12];
    uint32_t timestamp;
}STP23L_OneFrame_t;
#pragma pack()

static STP23L_OneFrame_t stp23L_frame;               //保存1帧stp23L数据，用来数据解析
static const uint8_t stp23L_BufLen = sizeof(stp23L_frame); //原始数据的长度
static uint8_t stp23L_buf[ stp23L_BufLen ] ;         //存储原始数据的数组
static uint8_t stp23L_counts = 0;                    //接收数据的计数值

//获取距离信息
//【注释】参数recv：本次喂进来的1个字节；参数dis：输出指针，只有成功解析出一帧才会被写入
void stp23L_getdistance_callback(uint8_t recv,float* dis)
{
	static uint8_t checkcode = 0;//校验码

    //定义状态机
    enum{
        Wait_HEAD=0       ,//等待格式数据阶段
        Handle_Ddata      ,//接收数据阶段
        END_DATA          ,//接收校验处理阶段
    };
	
	static uint8_t state_machine = Wait_HEAD;//初始化状态机,等待帧头
	switch( state_machine )
	{
		case Wait_HEAD://状态1：等待
		{
			static uint8_t headlen = 0;
			//前10字节数据均为已知的帧头：0xAA 0xAA 0xAA 0xAA 0x00 0x02 0x00 0x00 0xB8 0x00
			headlen++;
			if( 10 == headlen  ) 
			{
				headlen=0 , state_machine = Handle_Ddata , checkcode = 0xBA; //checkcode是去掉4个0xAA后开始累加,即算术初始值是 0x02+0xB8=0xBA
			}
			break;
		}
		case Handle_Ddata://状态2：接收
		{
			checkcode+=recv;
			stp23L_buf[stp23L_counts++] = recv;
			if( stp23L_BufLen == stp23L_counts ) stp23L_counts=0,state_machine = END_DATA;
			break;
		}

		case END_DATA://状态3：处理验证
		{
			if( recv == checkcode )
			{
				memcpy(&stp23L_frame,stp23L_buf,stp23L_BufLen);
				float  alldist = 0;
				for(uint8_t i=0;i<12;i++) alldist+=stp23L_frame.PointCloud[i].distance;
				*dis = alldist/12.0f; //距离信息求均值	
				*dis /= 1000.0f;      //单位由 mm 转为 m
      }
			else
			{
				//数据校验错误，*dis不会被更新，保持上一次的有效值
			}
			checkcode = 0;
			state_machine = Wait_HEAD;//再次回到状态1
			break;
		}
		
	}
}

