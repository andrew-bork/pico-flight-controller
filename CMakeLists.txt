cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


# rest of your project
include_directories(./include)
add_executable(a
./src/filter.cpp
./src/pid.cpp
./src/main.cpp
./src/bmp390.cpp
./src/i2c.cpp
./src/mpu6050.cpp
./src/quarternion.cpp
./src/vector.cpp
    )
target_link_libraries(a 
    hardware_i2c
    pico_i2c_slave
    pico_stdlib
    hardware_pwm
    pico_multicore)

pico_enable_stdio_usb(a 1)
pico_enable_stdio_uart(a 0)
# # create map/bin/hex/uf2 file in addition to ELF.
# pico_add_extra_outputs(a)