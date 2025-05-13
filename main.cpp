#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

// LCD function declarations (if written in C)
extern "C" {
    void lcd_init();
    void lcd_clear();
    void lcd_write_string(const char *str);
    void lcd_set_cursor(uint8_t row, uint8_t col);
}

#define I2C_PORT i2c0

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    lcd_init();
    lcd_clear();

    MPU6050 mpu(I2C_PORT);
    mpu.init();

    float accel[3], gyro[3];
    char line1[17], line2[17];

    while (true) {
        mpu.read(accel, gyro);

        snprintf(line1, sizeof(line1), "A:%.1f %.1f %.1f", accel[0], accel[1], accel[2]);
        snprintf(line2, sizeof(line2), "G:%.1f %.1f %.1f", gyro[0], gyro[1], gyro[2]);

        lcd_set_cursor(0, 0);
        lcd_write_string(line1);
        lcd_set_cursor(1, 0);
        lcd_write_string(line2);

        sleep_ms(200);
    }
}
