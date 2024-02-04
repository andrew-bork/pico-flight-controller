
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/i2c_slave.h>

#include <hardware/pwm.h>
#include <string.h>
#include <settings.h>
#include <hardware/i2c.h>
#include <pico/multicore.h>

#include <sensors/bmp390.hpp>
#include <sensors/mpu6050.hpp>

#include <math/quarternion.hpp>
#include <math/constants.hpp>

#include <pid.hpp>

#include <cmath>

float bmp_data[2];
float vertical_speed = 0.0;
float height = 0.0;
math::vector raw_accelerometer_acceleration, raw_gyroscope_angular_velocity;
math::vector accelerometer_acceleration, gyroscope_angular_velocity;
math::quarternion orientation(1, 0, 0, 0);
math::vector orientation_euler;

pid_controller roll_controller, pitch_controller;
float motor_powers[4] = { 0.0, 0.0, 0.0, 0.0 }; // fl, fr, bl, br
uint8_t motor_pins[4] = { 21, 20, 19, 18 };

float dt = 0.01;
float tau = 0.01;
float tau_z = 0.01;

float calculate_height(float temp_c, float pressure_k){
    float temp_k = temp_c + 273.15;

    // return (-1 + pow((p0/pressure_k),(1/5.255))) * temp_k / 0.0065;
    return - UNV_GAS_CONST * temp_k * log(pressure_k / PRESSURE_BENCHMARK) / (MOLAR_MASS_AIR * GRAVITATIONAL_ACCELERATION);
}

#define FLOAT_REGISTER_I2C(x) (uint8_t *)(&x),

uint8_t * mem_locations[] = {
    FLOAT_REGISTER_I2C(raw_accelerometer_acceleration.x) // 0x00
    FLOAT_REGISTER_I2C(raw_gyroscope_angular_velocity.x) // 0x01

    FLOAT_REGISTER_I2C(bmp_data[0]) // 0x02
    FLOAT_REGISTER_I2C(orientation_euler.x) // 0x03
    FLOAT_REGISTER_I2C(vertical_speed) //0x04
    FLOAT_REGISTER_I2C(height) // 0x05

    FLOAT_REGISTER_I2C(dt) // 0x06
    FLOAT_REGISTER_I2C(tau) // 0x07
};
static struct
{
    uint8_t * mem_address;
    bool mem_address_written;
} context;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = mem_locations[i2c_read_byte_raw(i2c)];
            context.mem_address_written = true;
        } else {
            // save into memory
            // context.mem[context.mem_address] = i2c_read_byte_raw(i2c);
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, *(context.mem_address));
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        break;
    default:
        break;
    }
}

void core1_main() {
    printf("init i2c1\n");
    
    i2c_inst_t * bus1 = &i2c1_inst;
    i2c_init(bus1, 100000);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c1_scl_pin);
    gpio_pull_up(i2c1_sda_pin);
    // configure I2C0 for slave mode
    i2c_slave_init(bus1, pico_i2c_address, &i2c_slave_handler);

    while(1) {
        printf("dt: %f s (%f hz), Roll: %.2f, Pitch: %.2f, Altitude: %.2f, Vr: %.2f\n", 
            dt, 
            1/dt, 
            orientation_euler.x * RAD_TO_DEG, 
            orientation_euler.y * RAD_TO_DEG, 
            height,
            raw_gyroscope_angular_velocity.x*RAD_TO_DEG);
        sleep_ms(10);
    }
}




math::vector calculate_roll_pitch(const math::vector& acceleration) {
    math::vector euler;
    euler.x = atan2(acceleration.y, acceleration.z);
    euler.y = atan2(-acceleration.x, sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
    return euler;
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("init i2c\n");
    i2c_inst_t * bus0 = &i2c0_inst;
    i2c_init(bus0, 100000);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c0_scl_pin);
    gpio_pull_up(i2c0_sda_pin);

    printf("init bmp\n");
    bmp390 bmp(bus0);

    bmp.soft_reset();
    bmp.set_oversample(bmp390::oversampling::STANDARD, bmp390::ULTRA_LOW_POWER);
    bmp.set_iir_filter(bmp390::COEFF_3);
    bmp.set_output_data_rate(bmp390::hz50);
    bmp.set_enable(true, true);
    
    bmp.set_enable_fifo(false, false);
    bmp.set_fifo_stop_on_full(false);

    bmp.set_pwr_mode(bmp390::NORMAL);

    printf("init mpu\n");
    mpu6050 mpu(bus0);
    
    mpu.set_accl_set(mpu6050::accl_range::g_2);
    mpu.set_gyro_set(mpu6050::gyro_range::deg_1000);
    mpu.set_clk(mpu6050::clk::y_gyro);
    mpu.set_fsync(mpu6050::fsync::input_dis);
    mpu.set_dlpf_bandwidth(mpu6050::dlpf::hz_94);
    mpu.wake_up();

    mpu.set_offsets(0.110017, 0.020430, -0.247031, -0.029376, 0.020785, 0.019256);


    for(int i = 0; i < 4; i ++) {
        gpio_set_function(motor_pins[i], GPIO_FUNC_PWM);
    }
    pwm_set_wrap(0, )

    roll_controller.kP = 0.01;
    pitch_controller.kP = 0.01;

    printf("finished init\n");

    uint64_t then = time_us_64();

    multicore_launch_core1(core1_main);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    while(1) {
        gpio_put(LED_PIN, 1);
        uint64_t now = time_us_64();
        dt = (now - then) * 0.0000001;
        then = now;

        mpu.get_data(raw_accelerometer_acceleration, raw_gyroscope_angular_velocity);
        bmp.get_data(bmp_data);

        accelerometer_acceleration = raw_accelerometer_acceleration;
        gyroscope_angular_velocity = raw_gyroscope_angular_velocity;
        // accelerometer_acceleration = accelerometer_filter[raw_accelerometer_acceleration];
        // gyroscope_angular_velocity = gyroscope_filter[raw_gyroscope_angular_velocity];

        float a_mag = math::length(accelerometer_acceleration);
        
        math::quarternion orientation_inverse = math::quarternion::conjugate(orientation);
        math::vector true_acceleration = math::quarternion::rotate_vector(orientation_inverse, accelerometer_acceleration);
        
        true_acceleration.z -= 9.81;
        float pressure_height = calculate_height(bmp_data[0], bmp_data[1]);

        vertical_speed = ((pressure_height - height) / dt) * tau_z + (1 - tau_z) * (vertical_speed + true_acceleration.z * dt);
        height = pressure_height * tau_z + (1 - tau_z) * (height + vertical_speed * dt);


        // Integrate angular velocity reading.
        math::quarternion gyro_quarternion = math::quarternion::from_euler_ZYX(gyroscope_angular_velocity*dt);
        orientation = gyro_quarternion * orientation;
        
        orientation_euler = math::quarternion::to_euler(orientation);
        math::vector accelerometer_orientation;

        // If the acceleration seems in range, use it to calculate orientation.
        accelerometer_orientation = calculate_roll_pitch(accelerometer_acceleration);
        accelerometer_orientation.z = orientation_euler.z;
        // Apply a complementary filter
        orientation_euler = accelerometer_orientation * tau + orientation_euler * (1 - tau);

        // Convert the fused angles back into the quarternion.
        orientation = math::quarternion::from_euler_ZYX(orientation_euler);
        
        gpio_put(LED_PIN, 0);
        sleep_ms(10);
        // printf("%5.2f %5.2f\n", orientation_euler.x, orientation_euler.y);
    }
}