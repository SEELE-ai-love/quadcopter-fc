#include "Bsp_iic.h"

/* ============================================================
 * 总线恢复：手动模拟I2C时钟脉冲，敲醒可能卡在"等待应答"状态的从设备
 * 触发场景：调试时下断点/复位/重新烧录，让STM32和ICM20948的通信在
 * 中途被打断，ICM20948还卡在等待后续时钟的状态，此后持续NACK(读到0xFF)，
 * 之前只能靠物理拔插电源解决，这里改成软件自动恢复。
 * ============================================================ */
#define I2C_RECOVERY_PORT   GPIOB
#define I2C_RECOVERY_SCL    GPIO_PIN_6
#define I2C_RECOVERY_SDA    GPIO_PIN_7

static void recovery_delay(void)
{
    for (volatile int i = 0; i < 200; i++) { __NOP(); }
}

void I2C_BusRecovery(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = I2C_RECOVERY_SCL | I2C_RECOVERY_SDA;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(I2C_RECOVERY_PORT, &gpio);

    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SDA, GPIO_PIN_SET);
    recovery_delay();

    /* 敲9个时钟脉冲，足够让卡住的从设备走完剩余的位、主动释放SDA */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL, GPIO_PIN_RESET);
        recovery_delay();
        HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL, GPIO_PIN_SET);
        recovery_delay();
    }

    /* 手动发STOP信号：SCL高电平期间，SDA从低变高，明确结束这次事务 */
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SDA, GPIO_PIN_RESET);
    recovery_delay();
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SCL, GPIO_PIN_SET);
    recovery_delay();
    HAL_GPIO_WritePin(I2C_RECOVERY_PORT, I2C_RECOVERY_SDA, GPIO_PIN_SET);
    recovery_delay();

    /* 这两个引脚之后会被MX_I2C1_Init()重新配置成I2C复用功能，这里不用改回去 */
}

/* ============================================================
 * 读写封装：加超时时间+失败重试，单次偶发失败不再直接放弃
 * ============================================================ */
uint8_t I2C_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        status = HAL_I2C_Mem_Write(hi2c, I2C_DEV_ADDR(dev_addr), reg,
                                    I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
        if (status == HAL_OK) break;
    }
    return status;
}

uint8_t I2C_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        status = HAL_I2C_Mem_Read(hi2c, I2C_DEV_ADDR(dev_addr), reg,
                                   I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT_MS);
        if (status == HAL_OK) break;
    }
    return status;
}

uint8_t I2C_WriteRegs(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg,
                       uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        status = HAL_I2C_Mem_Write(hi2c, I2C_DEV_ADDR(dev_addr), reg,
                                    I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS);
        if (status == HAL_OK) break;
    }
    return status;
}

uint8_t I2C_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg,
                      uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int i = 0; i < I2C_RETRY_COUNT; i++) {
        status = HAL_I2C_Mem_Read(hi2c, I2C_DEV_ADDR(dev_addr), reg,
                                   I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS);
        if (status == HAL_OK) break;
    }
    return status;
}