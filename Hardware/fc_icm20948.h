#ifndef FC_ICM20948_H
#define FC_ICM20948_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_ICM20948_ADDR_LOW   0x68U
#define FC_ICM20948_ADDR_HIGH  0x69U

#define FC_ICM20948_REG_BANK_SEL   0x7FU
#define FC_ICM20948_REG_WHO_AM_I   0x00U
#define FC_ICM20948_WHO_AM_I_VALUE 0xEAU

typedef int (*fc_i2c_read_fn)(uint8_t dev_addr, uint8_t reg,
                              uint8_t *buf, uint16_t len);
typedef int (*fc_i2c_write_fn)(uint8_t dev_addr, uint8_t reg,
                               const uint8_t *buf, uint16_t len);

typedef struct {
    uint8_t      dev_addr;   /* 7-bit I2C address */
    fc_i2c_read_fn  read;
    fc_i2c_write_fn write;
} fc_icm20948_t;

typedef struct {
    float ax;  /* m/s^2 */
    float ay;
    float az;
    float gx;  /* rad/s */
    float gy;
    float gz;
} fc_icm20948_data_t;

/* Verify the chip id and put the sensor in a known configuration. */
int fc_icm20948_init(fc_icm20948_t *dev);

/* Read accelerometer and gyroscope, scaled to SI units. */
int fc_icm20948_read(fc_icm20948_t *dev, fc_icm20948_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FC_ICM20948_H */
