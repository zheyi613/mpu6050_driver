/**
 * @file mpu6050.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief MPU6050 library for stm32f767zi
 * @date 2022-08-24
 */

#include "mpu6050.h"

static void calibration(uint16_t sample_rate);

static float gyro_unit = 0;
static float accel_unit = 0;
static int16_t gyro_bias[3] = {0};

/* Use PG2 as Vdd of MPU6050 */
void mpu6050_power_on(void)
{
	RCC->AHB1ENR |= GPIOG_CLK_EN;
	GPIOG->MODER |= GPIO_MODE_OUTPUT << 4;
	GPIOG->BSRR |= (1U << 2);

	systick_delay_ms(100);
}

/**
 * @brief Self-Test to get change from foctory trim of the self-test response.
 * 	  if |value| < 14, the MPU6050 keep its accuracy, else failed.
 *
 * @param err error array of gyro and accel change
 */
void mpu6050_selftest(float *err)
{
	uint8_t tmp = 0xE0;
	uint8_t raw_data[4] = {0};
	uint8_t selftest_data[6] = {0};
	float factory_trim = 0;

	// Configure the accelorometer and gyroscope self test
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &tmp);
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &tmp);

	systick_delay_ms(250);
	// Get raw data from self test register
	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_SELF_TEST_X, 4, raw_data);
	// Calculate self test data
	for (int i = 0; i < 3; i++) {
		selftest_data[i] = (raw_data[3] >> (2 * (2 - i))) & 0x3;
		selftest_data[i] |= (raw_data[i] & 0xE0) >> 3;
		selftest_data[3 + i] = raw_data[i] & 0x1F;
	}
	// Calculate change from factory trim of the Self-Test response
	for (int i = 0; i < 3; i++) {
		// accelerometer
		factory_trim =
			4096.0 * 0.34 *
			powf(0.92 / 0.34, ((float)selftest_data[i] - 1) / 30.0);
		err[i] = 100.0 +
			 100.0 * ((float)selftest_data[i] - factory_trim) /
				 factory_trim;
		// gyro factory trim
		factory_trim =
			25.0 * 131.0 * powf(1.046, selftest_data[3 + i] - 1);

		if (i == 1)
			factory_trim *= -1;

		err[3 + i] =
			100 +
			100.0 * ((float)selftest_data[3 + i] - factory_trim) /
				factory_trim;
	}

	// Clear configuration of the accelorometer and gyroscope self test
	tmp = 0;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &tmp);
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &tmp);
}

/*
 * Default setting:
 * Clock source: PLL with X axis gyroscope reference
 * Sampling rate: 1000
 * Gyroscope full scale range: 250dps
 * Accelerometer full scale range: 2g
 *
 * note: no calibration
 */
void mpu6050_default_init(void)
{
	uint8_t tmp = MPU6050_DEVICE_RESET;
	// Reset MPU6050
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1, &tmp);
	systick_delay_ms(100);
	// Wake up and use PLL with X axis gyroscope reference as clock source
	tmp = MPU6050_CLK_X_GYRO;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1, &tmp);
	systick_delay_ms(100);
	// Set sample rate to 1kHz
	tmp = 1;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_CONFIG, 1, &tmp);
	// initial gyroscope and accelorometer unit
	gyro_unit = MPU6050_GYRO_UNIT;
	accel_unit = MPU6050_ACCEL_UNIT;
}

/**
 * @brief Initial mpu6050 with user selecting sample rate,
 *        gyro full scale range, accel full scale range.
 *        Set proper low pass filter and calibrate mpu6050.
 *
 * @param sample_rate (Hz)
 * @param gyro_range  (dps)
 * @param accel_range (g)
 */
void mpu6050_init(uint16_t sample_rate, uint8_t accel_range,
		  uint16_t gyro_range)
{
	uint8_t div = (uint8_t)(1000 / sample_rate - 1);
	uint8_t low_pass_cfg = 0;
	uint16_t tmp = sample_rate;

	mpu6050_default_init();

	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 1, &div);
	// Set low pass filter
	while (1000 > tmp) {
		tmp <<= 1;
		low_pass_cfg++;
	}

	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_CONFIG, 1, &low_pass_cfg);
	// Set gyroscope and accelerometer full scale range
	tmp = 0;
	accel_range /= 4;

	while (accel_range) {
		accel_range >>= 1;
		accel_unit *= 2;
	}

	tmp <<= 8;
	gyro_range /= 500;

	while (gyro_range) {
		gyro_range >>= 1;
		gyro_unit *= 2;
		tmp++;
	}

	tmp <<= 3;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 2,
		      (uint8_t *)&tmp);
	systick_delay_ms(500);

	calibration(sample_rate);
}

static void calibration(uint16_t sample_rate)
{
	uint8_t tmp = MPU6050_USER_CTRL_FIFO_EN;
	uint16_t fifo_count = 0;
	uint16_t packet_count = 0;
	int16_t data[6] = {0};
	int32_t sum[6] = {0};
	float sum_squares = 0;

	// Enable FIFO
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_USER_CTRL, 1, &tmp);
	// Enable gyro and accel FIFO
	tmp = 0x78;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_FIFO_EN, 1, &tmp);
	// According to sample division to delay time
	systick_delay_ms(10000 / (int)sample_rate);
	// Disable accel and gyro FIFO
	tmp = 0;
	i2c_write_reg(I2C1, MPU6050_ADDRESS, MPU6050_FIFO_EN, 1, &tmp);
	// Read FIFO sample count
	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_FIFO_COUNTH, 2,
		     (uint8_t *)&fifo_count);
	fifo_count = (fifo_count << 8) | (fifo_count >> 8);
	packet_count = fifo_count / 12;
	// Read FIFO data and calculate gain/bias
	for (int i = 0; i < packet_count; i++) {
		i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_FIFO_R_W, 12,
			     (uint8_t *)data);

		for (int j = 0; j < 6; j++) {
			data[j] = (data[j] << 8) | ((data[j] >> 8) & 0xFF);
			sum[j] += (int32_t)data[j];
		}
	}

	for (int i = 0; i < 3; i++) {
		sum[i] /= packet_count;
		sum_squares += (float)sum[i] * (float)sum[i];
		gyro_bias[i] = (int16_t)(sum[i + 3] / packet_count);
	}
	// Change accel unit by accel gain
	accel_unit *= 16384.0 / sqrtf(sum_squares);
}

void mpu6050_kalman(float (*X)[2], float *angle, float *rate, float dt)
{
	static float P[2][2][2] = {0}, Q = 0.08, R = 0.5;
	float X_[2] = {0}, P_[2][2] = {0}, K[2] = {0};

	for (int i = 0; i < 2; i++) {
		X_[0] = X[i][0] + dt * (rate[i] - X[i][1]);
		X_[1] = X[i][1];

		P_[0][0] =
			P[i][0][0] +
			(-P[i][0][1] - P[i][1][0] + P[i][1][1] * dt + Q) * dt;
		P_[0][1] = P[i][0][1] - P[i][1][1] * dt;
		P_[1][0] = P[i][1][0] - P[i][1][1] * dt;
		P_[1][1] = P[i][1][1] + Q * dt;

		K[0] = P_[0][0] / (P_[0][0] + R);
		K[1] = P_[1][0] / (P_[0][0] + R);

		X[i][0] = X_[0] + K[0] * (angle[i] - X[i][0]);
		X[i][1] = X_[1] + K[1] * (angle[i] - X[i][0]);

		P[i][0][0] = P_[0][0] - K[0] * P_[0][0];
		P[i][0][1] = P_[0][1] - K[0] * P_[0][1];
		P[i][1][0] = P_[1][0] - K[1] * P_[0][0];
		P[i][1][1] = P_[1][1] - K[1] * P_[0][1];
	}
	// Yaw can't use filter to decrease noise, so use threshold to eliminate
	if (rate[2] < 0.08 && rate[2] > -0.08)
		rate[2] = 0;
	X[2][0] += dt * rate[2];
	X[2][1] = rate[2];
}

/**
 * @brief get static attitude by measuring gravity of accelerometer
 *
 * @param accel x, y, z
 * @param euler roll, pitch, yaw(unknown)
 */
void mpu6050_static_attitude(float *accel, float *euler)
{
	euler[0] = atan2f(accel[1], accel[2]) * 180 / PI;
	euler[1] = -atan2f(accel[0],
			   sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) *
		   	   180 / PI;
}

/**
 * @brief data from i2c read function: X_H > X_L > Y_H > Y_L > Z_H > Z_L
 * 	  reverse order: X_L > X_H > Y_L > Y_H > Z_L > Z_H
 * 	  int16 data order: X > Y > Z
 *
 * @param data
 * 	  accelerometer: [0:2]
 * 	  temperature: [3]
 * 	  gyroscope: [4:6]
 */
void mpu6050_get_all(float *data)
{
	int16_t tmp[7] = {0};

	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUTH, 14,
		     (uint8_t *)tmp);

	for (int i = 0; i < 7; i++) {
		tmp[i] = (tmp[i] << 8) | ((tmp[i] >> 8) & 0xFF);

		if (i < 3) {
			// tmp[i] -= gyro_bias[i];
			data[i] = (float)tmp[i] * accel_unit;
		} else if (i == 3) {
			data[i] = (float)tmp[i] * 0.00294117647 + 36.53;
		} else {
			tmp[i] -= gyro_bias[i - 4];
			data[i] = (float)tmp[i] * gyro_unit;
		}
	}
}

void mpu6050_get_accel(float *data)
{
	int16_t tmp[3] = {0};

	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUTH, 6,
		     (uint8_t *)tmp);

	for (int i = 0; i < 3; i++) {
		tmp[i] = (tmp[i] << 8) | ((tmp[i] >> 8) & 0xFF);
		data[i] = (float)tmp[i] * accel_unit;
	}
}

/*
 * Temperature in degree C = (TEMP_OUT register value as a signed quantity) /
 * 340 + 36.53
 */
float mpu6050_get_temperature(void)
{
	int16_t tmp = 0;

	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_TEMP_OUTH, 2,
		     (uint8_t *)&tmp);
	tmp = (tmp << 8) | ((tmp >> 8) & 0xFF);
	return (float)tmp * 0.00294117647 + 36.53;
}

void mpu6050_get_gyro(float *data)
{
	int16_t tmp[3] = {0};

	i2c_read_reg(I2C1, MPU6050_ADDRESS, MPU6050_GYRO_XOUTH, 6,
		     (uint8_t *)&tmp);

	for (int i = 0; i < 3; i++) {
		tmp[i] = (tmp[i] << 8) | ((tmp[i] >> 8) & 0xFF);
		tmp[i] -= gyro_bias[i];
		data[i] = (float)tmp[i] * gyro_unit;
	}
}