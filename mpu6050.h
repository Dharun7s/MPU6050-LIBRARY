#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY  131.0f

class MPU6050 {
public:
    MPU6050(i2c_inst_t *i2c_port);

    void init();
    void read(float accel[3], float gyro[3]);

private:
    i2c_inst_t *i2c;
};

#endif
