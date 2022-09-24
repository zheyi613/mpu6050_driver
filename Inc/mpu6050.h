/**
 * @file mpu6050.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief mpu6050 library for stm32f767zi
 * @date 2022-08-24
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f767zi_i2c.h"
#include "stm32f767zi_systick.h"
#include "math.h"

#define PI acos(-1)
/*
 * Line Connection:
 * PG2 (3.16V, 25mA) <----> Vdd (2.375 < voltage < 3.46, current > 3.9mA)
 *               GND <----> GND
 *         PB6 (SCL) <----> SCL
 *         PB9 (SDA) <----> SDA
 *               GND <----> AD0
 * 
 * 1. SDA and SCL lines typically need pull-up resistors to Vdd.
 * 2. Maximum I2C speed: 400kHz
 * 3. The slave address: 0b110100x
 *    (The LSB of address is determined by the logic level on pin AD0.)
 * 4. Gyro output speed: 8kHz, Accelerometer output rate:1kHz
 * 5. The MPU-60X0 automatically increments the register address and
 *    and loads the data to the appropriate register.
 */
#define MPU6050_AD0                     0
#define MPU6050_ADDRESS                 (0b1101000 | MPU6050_AD0)

#define MPU6050_XG_OFFS_TC		0x00	// [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC		0x01	// [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC		0x02	// [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN		0x03
#define MPU6050_Y_FINE_GAIN		0x04
#define MPU6050_Z_FINE_GAIN		0x05
#define MPU6050_XA_OFFS_H		0x06
#define MPU6050_XA_OFFS_L_TC		0x07
#define MPU6050_YA_OFFS_H		0x08
#define MPU6050_YA_OFFS_L_TC		0x09
#define MPU6050_ZA_OFFS_H		0x0A
#define MPU6050_ZA_OFFS_L_TC		0x0B
#define MPU6050_SELF_TEST_X             0x0D
#define MPU6050_SELF_TEST_Y             0x0E
#define MPU6050_SELF_TEST_Z             0x0F
#define MPU6050_SELF_TEST_A             0x10
#define MPU6050_XG_OFFS_USRH		0x13
#define MPU6050_XG_OFFS_USRL		0x14
#define MPU6050_YG_OFFS_USRH		0x15
#define MPU6050_YG_OFFS_USRL		0x16
#define MPU6050_ZG_OFFS_USRH		0x17
#define MPU6050_ZG_OFFS_USRL		0x18
#define MPU6050_SMPLRT_DIV              0x19
#define MPU6050_CONFIG                  0x1A
#define MPU6050_GYRO_CONFIG             0x1B
#define MPU6050_ACCEL_CONFIG            0x1C
#define MPU6050_FIFO_EN                 0x23
#define MPU6050_I2C_MST_CTRL            0x24
#define MPU6050_I2C_SLV0_ADDR           0x25
#define MPU6050_I2C_SLV0_REG            0x26
#define MPU6050_I2C_SLV0_CTRL           0x27
#define MPU6050_I2C_SLV1_ADDR           0x28
#define MPU6050_I2C_SLV1_REG            0x29
#define MPU6050_I2C_SLV1_CTRL           0x2A
#define MPU6050_I2C_SLV2_ADDR           0x2B
#define MPU6050_I2C_SLV2_REG            0x2C
#define MPU6050_I2C_SLV2_CTRL           0x2D
#define MPU6050_I2C_SLV3_ADDR           0x2E
#define MPU6050_I2C_SLV3_REG            0x2F
#define MPU6050_I2C_SLV3_CTRL           0x30
#define MPU6050_I2C_SLV4_ADDR           0x31
#define MPU6050_I2C_SLV4_REG            0x32
#define MPU6050_I2C_SLV4_DO             0x33
#define MPU6050_I2C_SLV4_CTRL           0x34
#define MPU6050_I2C_SLV4_DI             0x35
#define MPU6050_I2C_MST_STATUS          0x36
#define MPU6050_INT_PIN_CFG             0x37
#define MPU6050_INT_ENABLE              0x38
#define MPU6050_INT_STATUS              0x3A
#define MPU6050_ACCEL_XOUTH             0x3B    // 2 bytes
#define MPU6050_ACCEL_XOUTL             0x3C
#define MPU6050_ACCEL_YOUTH             0x3D    // 2 bytes
#define MPU6050_ACCEL_YOUTL             0x3E
#define MPU6050_ACCEL_ZOUTH             0x3F    // 2 bytes
#define MPU6050_ACCEL_ZOUTL             0x40
#define MPU6050_TEMP_OUTH               0x41    // 2 bytes
#define MPU6050_TEMP_OUTL               0x42
#define MPU6050_GYRO_XOUTH              0x43    // 2 bytes
#define MPU6050_GYRO_XOUTL              0x44
#define MPU6050_GYRO_YOUTH              0x45    // 2 bytes
#define MPU6050_GYRO_YOUTL              0x46
#define MPU6050_GYRO_ZOUTH              0x47    // 2 bytes
#define MPU6050_GYRO_ZOUTL              0x48
#define MPU6050_EXT_SENS_DATA_01        0x49
#define MPU6050_EXT_SENS_DATA_02        0x4A
#define MPU6050_EXT_SENS_DATA_03        0x4B
#define MPU6050_EXT_SENS_DATA_04        0x4C
#define MPU6050_EXT_SENS_DATA_05        0x4D
#define MPU6050_EXT_SENS_DATA_06        0x4E
#define MPU6050_EXT_SENS_DATA_07        0x4F
#define MPU6050_EXT_SENS_DATA_08        0x50
#define MPU6050_EXT_SENS_DATA_09        0x51
#define MPU6050_EXT_SENS_DATA_10        0x52
#define MPU6050_EXT_SENS_DATA_11        0x53
#define MPU6050_EXT_SENS_DATA_12        0x54
#define MPU6050_EXT_SENS_DATA_13        0x55
#define MPU6050_EXT_SENS_DATA_14        0x56
#define MPU6050_EXT_SENS_DATA_15        0x57
#define MPU6050_EXT_SENS_DATA_16        0x58
#define MPU6050_EXT_SENS_DATA_17        0x59
#define MPU6050_EXT_SENS_DATA_18        0x5A
#define MPU6050_EXT_SENS_DATA_19        0x5B
#define MPU6050_EXT_SENS_DATA_20        0x5C
#define MPU6050_EXT_SENS_DATA_21        0x5D
#define MPU6050_EXT_SENS_DATA_22        0x5E
#define MPU6050_EXT_SENS_DATA_23        0x5F
#define MPU6050_I2C_SLV0_DO             0x63
#define MPU6050_I2C_SLV1_DO             0x64
#define MPU6050_I2C_SLV2_DO             0x65
#define MPU6050_I2C_SLV3_DO             0x66
#define MPU6050_I2C_MST_DELAY_CTRL      0x67
#define MPU6050_SIGNAL_PATH_RESET       0x68
#define MPU6050_USER_CTRL               0x6A
#define MPU6050_PWR_MGMT_1              0x6B
#define MPU6050_PWR_MGMT_2              0x6C
#define MPU6050_FIFO_COUNTH             0x72    // 2 bytes
#define MPU6050_FIFO_COUNTL		0x73
#define MPU6050_FIFO_R_W                0x74
#define MPU6050_WHO_AM_I                0x75   
// User Control
#define MPU6050_USER_CTRL_FIFO_EN	(1U << 6)    
// Power Management 1
#define MPU6050_CLK_INTERNAL            0U
#define MPU6050_CLK_X_GYRO              1U
#define MPU6050_CLK_Y_GYRO              2U
#define MPU6050_CLK_Z_GYRO              3U
#define MPU6050_CLK_EXTERNAL1           4U      // 32.768kHz
#define MPU6050_CLK_EXTERNAL2           5U      // 19.2MHz
#define MPU6050_CLK_STOP                7U
#define MPU6050_TEMP_DISABLE            (1U << 3)
#define MPU6050_CYCLE                   (1U << 5)
#define MPU6050_SLEEP                   (1U << 6)
#define MPU6050_DEVICE_RESET            (1U << 7)
// Scale range unit
#define MPU6050_GYRO_UNIT               0.007629510948F // minimum unit (+-250dps)
#define MPU6050_ACCEL_UNIT              0.000061036087F // minimum unit (+-2g)

void mpu6050_power_on(void);
void mpu6050_selftest(float *err);
void mpu6050_default_init(void);
void mpu6050_init(uint16_t sample_rate, uint8_t acce_range, uint16_t gyro_range);
void mpu6050_kalman(float (*X)[2], float *angle, float *rate, float dt);
void mpu6050_static_attitude(float *accel, float *euler);
void mpu6050_get_all(float *data);
float mpu6050_get_temperature(void);
void mpu6050_get_gyro(float *data);
void mpu6050_get_accel(float *data);

#endif