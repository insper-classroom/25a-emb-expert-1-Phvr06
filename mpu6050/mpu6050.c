#include "mpu6050.h"

void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

int mpu6050_init(imu_c config) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    mpu6050_reset(config);

    return 0;
}

int mpu6050_reset(imu_c config) {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(config.i2c, MPU6050_ADDR, buf, 2, false);

    return 0;
}

int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buf[6];

    i2c_write_blocking(config.i2c, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(config.i2c, MPU6050_ADDR, buf, 6, false);
    
    for (int i = 0; i < 3; i++) {
        accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    return 0;
}

int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t reg = MPU6050_REG_GYRO_XOUT_H;
    uint8_t buf[6];

    i2c_write_blocking(config.i2c, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(config.i2c, MPU6050_ADDR, buf, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    return 0;
}

int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t reg = MPU6050_REG_TEMP_OUT_H;
    uint8_t buf[2];

    i2c_write_blocking(config.i2c, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(config.i2c, MPU6050_ADDR, buf, 2, false); // False - finished with bus
    int16_t rawTemp = (buf[0] << 8 | buf[1]);
    *temp = (rawTemp / 340.0) + 36.53;
    
    return 0;
}