#include "mpu6050.h"

MPU6050::MPU6050(i2c_inst_t *i2c_port) {
    i2c = i2c_port;
}

void MPU6050::init() {
    uint8_t buf[2];

    buf[0] = PWR_MGMT_1;
    buf[1] = 0x00;
    i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);

    buf[0] = ACCEL_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);

    buf[0] = GYRO_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);
}

void MPU6050::read(float accel[3], float gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = ACCEL_XOUT_H;

    i2c_write_blocking(i2c, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, MPU6050_ADDR, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        int16_t raw = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        accel[i] = raw / ACCEL_SENSITIVITY;
    }

    for (int i = 0; i < 3; i++) {
        int16_t raw = (buffer[8 + i * 2] << 8) | buffer[9 + i * 2];
        gyro[i] = raw / GYRO_SENSITIVITY;
    }
}
