#pragma once

#define MPU6050_DEFAULT_ADDR 0x68

#include <sensors/i2c.hpp>
#include <math/vector.hpp>
struct mpu6050 {
	/**
	 * @brief Accelerometer range. g_2 means +- 2 gs. g = 9.81 m/s^2
	 * 
	 */
	enum accl_range {
		g_2 = 0b00,
		g_4 = 0b01,
		g_8 = 0b10,
		g_16 = 0b11,
	};

	/**
	 * @brief Gyroscope range. deg_250 means +- 250 degrees
	 * 
	 */
	enum gyro_range {
		deg_250 = 0b00,
		deg_500 = 0b01,
		deg_1000 = 0b10,
		deg_2000 = 0b11,
	};

	/**
	 * @brief i have no clue
	 * 
	 */
	enum fsync {
		input_dis,
		temp_out_l,
		gyro_x_out_l,
		gyro_y_out_l,
		gyro_z_out_l,
		accl_x_out_l,
		accl_y_out_l,
		accl_z_out_l,
	};

	/**
	 * @brief Sets the coefficient of the internal low pass filter.
	 * 
	 */
	enum dlpf {
		hz_260,
		hz_184,
		hz_94,
		hz_44,
		hz_21,
		hz_10,
		hz_5, 
	};

	/**
	 * @brief Sets the source of the mpu6050's internal clock
	 * 
	 */
	enum clk {
		int_oscl,
		x_gyro,
		y_gyro,
		z_gyro,
		ext_32kHz,
		ext_19MHz,
		reserved,
		stop
	};


	mpu6050(i2c_inst_t * bus, int addr=MPU6050_DEFAULT_ADDR);
	~mpu6050();

	bool is_awake();
	void wake_up();
	void sleep();

	void set_accl_set(accl_range set);
	void set_gyro_set(gyro_range set);
	void set_clk(clk set);
	void set_dlpf_bandwidth(dlpf set);
	void set_fsync(fsync set);
	
	void set_offsets(float x_a, float y_a, float z_a, float x_g, float y_g, float z_g);

	void get_data_raw(int16_t * data);
	
	void get_data_wo_offsets(float * data);

	void get_data(float * data);
	void get_data(math::vector& acceleration, math::vector& angular_velocity);
	

	int query_register(int reg);
	void set_register(int reg, int data);

	void calibrate(int n);

	void print_debug();


	float offsets[6];
	
	private:
		float accel_scale = 1.0, gyro_scale = 1.0;
		i2c::device device;
};