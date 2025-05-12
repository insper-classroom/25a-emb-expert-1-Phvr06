#include "mpu6050.h"
#include <stdio.h>

int main() {
    stdio_init_all();

    // MPU
    imu_c imu_config;

    // geral
    mpu6050_set_config(&imu_config, i2c_default, 4, 5, 0);
    mpu6050_init(imu_config);

    while (1) {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t tmp;

        mpu6050_read_acc(imu_config, accel);
        mpu6050_read_gyro(imu_config, gyro);
        mpu6050_read_temp(imu_config, &tmp);

        printf("Acelerômetro: X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
        printf("Giroscópio: X: %d, Y: %d, Z: %d\n", gyro[0], gyro[1], gyro[2]);
        printf("Temperatura: %d\n", tmp);
        sleep_ms(1000);
    }

    return 0;
}