#include "fc_icm20948.h"

#define BANK_SEL_BANK0  0x00U
#define BANK_SEL_BANK2  0x20U

#define REG_PWR_MGMT_1      0x06U   /* bank 0 */
#define REG_USER_CTRL       0x03U   /* bank 0 */
#define REG_GYRO_XOUT_H     0x33U   /* bank 0 */
#define REG_ACCEL_XOUT_H    0x2DU   /* bank 0 */
#define REG_GYRO_CONFIG_1   0x01U   /* bank 2 */
#define REG_ACCEL_CONFIG_1  0x14U   /* bank 2 */

/* Scale factors for default ranges after this init (gyro +-250 dps,
   accel +-2 g). */
#define GYRO_LSB_TO_RAD      0.0001331f
#define ACCEL_LSB_TO_MS2     0.0005986f  /* 9.80665 / 16384 */

static int set_bank(fc_icm20948_t *dev, uint8_t bank)
{
    uint8_t val = bank;
    return dev->write(dev->dev_addr, FC_ICM20948_REG_BANK_SEL, &val, 1U);
}

static int16_t read_s16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

int fc_icm20948_init(fc_icm20948_t *dev)
{
    if ((dev == 0) || (dev->read == 0) || (dev->write == 0)) {
        return -1;
    }

    uint8_t who = 0;
    if (dev->read(dev->dev_addr, FC_ICM20948_REG_WHO_AM_I, &who, 1U) != 0) {
        return -2;
    }
    if (who != FC_ICM20948_WHO_AM_I_VALUE) {
        return -3;
    }

    /* Reset, then clear sleep and select the PLL clock source. */
    uint8_t pwr = 0x81U;
    if (dev->write(dev->dev_addr, REG_PWR_MGMT_1, &pwr, 1U) != 0) {
        return -4;
    }
    pwr = 0x01U;
    if (dev->write(dev->dev_addr, REG_PWR_MGMT_1, &pwr, 1U) != 0) {
        return -5;
    }

    /* Configure gyro and accel ranges in bank 2. */
    if (set_bank(dev, BANK_SEL_BANK2) != 0) {
        return -6;
    }
    uint8_t cfg = 0x00U;   /* gyro +-250 dps, no low-pass */
    if (dev->write(dev->dev_addr, REG_GYRO_CONFIG_1, &cfg, 1U) != 0) {
        return -7;
    }
    cfg = 0x00U;           /* accel +-2 g */
    if (dev->write(dev->dev_addr, REG_ACCEL_CONFIG_1, &cfg, 1U) != 0) {
        return -8;
    }
    if (set_bank(dev, BANK_SEL_BANK0) != 0) {
        return -9;
    }

    return 0;
}

int fc_icm20948_read(fc_icm20948_t *dev, fc_icm20948_data_t *out)
{
    if ((dev == 0) || (out == 0) || (dev->read == 0)) {
        return -1;
    }

    /* Accel and gyro blocks are not contiguous; the temperature register sits
       between them, so read each block separately. */
    uint8_t acc_raw[6];
    uint8_t gyro_raw[6];
    if (dev->read(dev->dev_addr, REG_ACCEL_XOUT_H, acc_raw, sizeof(acc_raw)) != 0) {
        return -2;
    }
    if (dev->read(dev->dev_addr, REG_GYRO_XOUT_H, gyro_raw, sizeof(gyro_raw)) != 0) {
        return -3;
    }

    int16_t ax = read_s16(acc_raw + 0);
    int16_t ay = read_s16(acc_raw + 2);
    int16_t az = read_s16(acc_raw + 4);
    int16_t gx = read_s16(gyro_raw + 0);
    int16_t gy = read_s16(gyro_raw + 2);
    int16_t gz = read_s16(gyro_raw + 4);

    out->ax = (float)ax * ACCEL_LSB_TO_MS2;
    out->ay = (float)ay * ACCEL_LSB_TO_MS2;
    out->az = (float)az * ACCEL_LSB_TO_MS2;
    out->gx = (float)gx * GYRO_LSB_TO_RAD;
    out->gy = (float)gy * GYRO_LSB_TO_RAD;
    out->gz = (float)gz * GYRO_LSB_TO_RAD;
    return 0;
}
