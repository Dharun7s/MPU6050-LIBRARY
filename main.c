#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// LCD Configuration
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// I2C Port
#define I2C_PORT i2c0

// Sensor sensitivity
#define ACCEL_SENSITIVITY 16384.0f  // ±2g range
#define GYRO_SENSITIVITY  131.0f    // ±250°/s range

// Function prototypes
void mpu6050_init();
void mpu6050_read_data(float accel[3], float gyro[3]);
void lcd_init();
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_write_string(const char *str);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear();

int main() {
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C); // SDA
    gpio_set_function(1, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Initialize devices
    lcd_init();
    mpu6050_init();
    lcd_clear();

    float accel[3], gyro[3];
    char line1[17], line2[17]; // 16 chars + null terminator

    while (1) {
        mpu6050_read_data(accel, gyro);

        // Format first line: Ax, Ay, Az
        snprintf(line1, sizeof(line1), "A:%.1f %.1f %.1f", 
                accel[0], accel[1], accel[2]);
        
        // Format second line: Gx, Gy, Gz
        snprintf(line2, sizeof(line2), "G:%.1f %.1f %.1f", 
                gyro[0], gyro[1], gyro[2]);

        // Update display
        lcd_set_cursor(0, 0);
        lcd_write_string(line1);
        lcd_set_cursor(1, 0);
        lcd_write_string(line2);

        sleep_ms(200); // Update every 200ms
    }

    return 0;
}

void mpu6050_init() {
    uint8_t buf[2];
    
    // Wake up MPU6050
    buf[0] = PWR_MGMT_1;
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    
    // Configure accelerometer (±2g)
    buf[0] = ACCEL_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    
    // Configure gyroscope (±250°/s)
    buf[0] = GYRO_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_data(float accel[3], float gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = ACCEL_XOUT_H;
    
    // Read all sensor data
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);
    
    // Convert accelerometer data (to g)
    for (int i = 0; i < 3; i++) {
        int16_t raw = (buffer[i*2] << 8) | buffer[i*2 + 1];
        accel[i] = raw / ACCEL_SENSITIVITY;
    }
    
    // Convert gyroscope data (to °/s)
    for (int i = 0; i < 3; i++) {
        int16_t raw = (buffer[i*2 + 8] << 8) | buffer[i*2 + 9];
        gyro[i] = raw / GYRO_SENSITIVITY;
    }
}

// LCD Functions (remain unchanged from previous example)
void lcd_init() {
    sleep_ms(50);
    lcd_send_cmd(0x03);
    sleep_ms(5);
    lcd_send_cmd(0x03);
    sleep_us(100);
    lcd_send_cmd(0x03);
    sleep_us(100);
    lcd_send_cmd(0x02);
    sleep_us(100);
    lcd_send_cmd(0x28);
    sleep_us(50);
    lcd_send_cmd(0x0C);
    sleep_us(50);
    lcd_send_cmd(0x01);
    sleep_ms(2);
    lcd_send_cmd(0x06);
    sleep_us(50);
}

void lcd_clear() {
    lcd_send_cmd(0x01);
    sleep_ms(2);
}

void lcd_send_cmd(uint8_t cmd) {
    uint8_t buf[4];
    buf[0] = (cmd & 0xF0) | 0x04;
    buf[1] = (cmd & 0xF0) | 0x00;
    buf[2] = ((cmd << 4) & 0xF0) | 0x04;
    buf[3] = ((cmd << 4) & 0xF0) | 0x00;
    i2c_write_blocking(I2C_PORT, LCD_ADDR, buf, 4, false);
}

void lcd_send_data(uint8_t data) {
    uint8_t buf[4];
    buf[0] = (data & 0xF0) | 0x05;
    buf[1] = (data & 0xF0) | 0x01;
    buf[2] = ((data << 4) & 0xF0) | 0x05;
    buf[3] = ((data << 4) & 0xF0) | 0x01;
    i2c_write_blocking(I2C_PORT, LCD_ADDR, buf, 4, false);
}

void lcd_write_string(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_send_cmd(address);
}
