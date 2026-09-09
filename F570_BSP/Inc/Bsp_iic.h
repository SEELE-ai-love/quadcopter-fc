#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "i2c.h"
#include <stdint.h>

#define I2C_DEV_ADDR(addr)  ((addr) << 1)

/* I2C超时时间(ms)，比之前的100ms稍微给宽松一点，避免总线偶尔慢一点就误判失败 */
#define I2C_TIMEOUT_MS  200

/* 读写重试次数：单次通信失败不代表总线真的死了，允许重试几次再放弃 */
#define I2C_RETRY_COUNT 3

uint8_t I2C_WriteReg (I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t I2C_ReadReg  (I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t *data);
uint8_t I2C_WriteRegs(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t I2C_ReadRegs (I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);

/* 总线恢复：手动敲9个时钟脉冲把可能卡死的从设备"敲醒"，
 * 解决"从设备通信中途被打断，一直不应答，读出来全是0xFF"这个问题。
 * 建议在MX_I2C1_Init()之前调用一次。 */
void I2C_BusRecovery(void);

#endif