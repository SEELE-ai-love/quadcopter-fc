#ifndef __BSP_STP23L_H
#define __BSP_STP23L_H

#include <stdint.h>

//stp23L激光模块 DMA数据缓冲区长度
#define userconfig_STP23L_DMABUF_LEN 200

//stp23L sensor的原始数据
typedef struct {
	uint8_t Buf[userconfig_STP23L_DMABUF_LEN];//DMA数据缓冲区
	uint16_t DataLen;//DMA接收的字节
}OriData_STP23L_t;

//STP23L数据处理和输出接口
extern float g_readonly_distance;//这个就是高度
extern OriData_STP23L_t DMABuf_oridata_stp23L;// DMABuf_oridata_stp23L包含结构体的两个成员。一个DataLen，另一个Buf，是DMA搬运的数据

//必须按顺序、一个不漏地喂字节，不能一次性把一整包数据丢给它
//只有当状态机走到END_DATA阶段且校验通过时，才会真正写入*dis，
//否则*dis保持调用前的值不变——这是天然的"数据无效就不更新"的容错机制
void stp23L_getdistance_callback(uint8_t recv,float* dis);
#endif /* __BSP_STP23L_H */
