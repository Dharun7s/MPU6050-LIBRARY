cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(mpu6050_project C CXX)  # C++ enabled
pico_sdk_init()

add_executable(mpu6050_project
    main.cpp
    mpu6050.cpp
    lcd_i2c.c  # <- If LCD code is in C
)

target_link_libraries(mpu6050_project pico_stdlib hardware_i2c)
pico_add_extra_outputs(mpu6050_project)
