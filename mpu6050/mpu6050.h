#ifndef _MPU6050_H
#define _MPU6050_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define MPU6050_ADDR 0x68

#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_ACCEL_CONFIG  0x1C

typedef struct imu6050 {
    // configuração do I2C
    i2c_inst_t *i2c;
    int pin_sda;
    int pin_scl;

    // configuração do range do acelerômetro
    int acc_scale;
} imu_c;

void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale);

int mpu6050_init(imu_c config);

int mpu6050_reset(imu_c config);

int mpu6050_read_acc(imu_c config, int16_t accel[3]);

int mpu6050_read_gyro(imu_c config, int16_t gyro[3]);

int mpu6050_read_temp(imu_c config, int16_t *temp);

#endif