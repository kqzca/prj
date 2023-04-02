/*
 * common.c
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#include "stm32f1xx_hal.h"
#include "mpu.h"

/*
HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data) {
	uint8_t tx_buf[2] = {reg, data};
	return HAL_I2C_Master_Transmit(hi2c, i2c_addr, tx_buf, 2, 1);
}

HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buf) {

	reg |= 0x80; // Add read bit
	if (len > 1) {

		reg |= 0x40; // Add autoincrement bit
	}
	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(hi2c, i2c_addr, &reg, 1, 1);
	if (res != HAL_OK) {
		return res;
	}
	return HAL_I2C_Master_Receive(hi2c, i2c_addr, buf, len, 1);
}
*/

static uint8_t i2c_read_buffer[6];
static const MPU_CONFIG mpu_config = {
  MPU6XXX_ACCEL_RANGE_2G,
  MPU6XXX_GYRO_RANGE_250DPS,
  MPU6XXX_DLPF_DISABLE,
  MPU6XXX_SLEEP_DISABLE,
  8000
};
static const uint16_t MPU6XXX_ACCEL_SEN = 16384;
static const uint16_t MPU6XXX_GYRO_SEN = 1310;
static uint16_t accel_sen = MPU6XXX_ACCEL_SEN;
static uint16_t gyro_sen = MPU6XXX_GYRO_SEN;

inline HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1);
}

inline HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buffer) {
	return HAL_I2C_Mem_Read(hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 1);
}

inline HAL_StatusTypeDef mpu_get_accel_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, XYZ_INT16T *xyz) {
  return i2c_read_regs(hi2c, i2c_addr, MPU6XXX_RA_ACCEL_XOUT_H, 6, i2c_read_buffer);
}

void mpu_get_accel(uint8_t *buf_6_bytes, XYZ_INT16T *xyz) {
  int16_t x = ((uint16_t)buf_6_bytes[0] << 8) + buf_6_bytes[1];
  int16_t y = ((uint16_t)buf_6_bytes[2] << 8) + buf_6_bytes[3];
  int16_t z = ((uint16_t)buf_6_bytes[4] << 8) + buf_6_bytes[5];

  xyz->x = (int32_t)x * 1000 / accel_sen;
  xyz->y = (int32_t)y * 1000 / accel_sen;
  xyz->z = (int32_t)z * 1000 / accel_sen;
}

HAL_StatusTypeDef mpu_get_gyro_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, XYZ_INT16T *xyz) {
	return i2c_read_regs(hi2c, i2c_addr, MPU6XXX_RA_GYRO_XOUT_H, 6, i2c_read_buffer);
}

void mpu_get_gyro(uint8_t *buf_6_bytes, XYZ_INT16T *xyz) {
  int16_t x = ((uint16_t)buf_6_bytes[0] << 8) + buf_6_bytes[1];
  int16_t y = ((uint16_t)buf_6_bytes[2] << 8) + buf_6_bytes[3];
  int16_t z = ((uint16_t)buf_6_bytes[4] << 8) + buf_6_bytes[5];

  xyz->x = (int32_t)x * 100 / gyro_sen;
  xyz->y = (int32_t)y * 100 / gyro_sen;
  xyz->z = (int32_t)z * 100 / gyro_sen;
}

// If needed, MPU6XXX_RA_XA_OFFS_H for getting accel offset and MPU6XXX_RA_XG_OFFS_USRH for gyro offset

static HAL_StatusTypeDef i2c_read_bit(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t bit, uint8_t *data)
{
    uint8_t byte;
    HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, reg, 1, &byte);
    if (res != HAL_OK) {
        return res;
    }

    *data = byte & (1 << bit);

    return HAL_OK;
}

static HAL_StatusTypeDef i2c_write_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t data)
{
    uint8_t byte, mask;
    HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, reg, 1, &byte);
    if (res != HAL_OK) {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    data <<= (start_bit - len + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte

    return i2c_write_reg(hi2c, i2c_addr, reg, byte);
}

static HAL_StatusTypeDef i2c_read_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t *data)
{
    uint8_t byte, mask;
    HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, reg, 1, &byte);
    if (res != HAL_OK) {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    byte &= mask;
    byte >>= (start_bit - len + 1);
    *data = byte;

    return HAL_OK;
}

static HAL_StatusTypeDef mpu_set_param(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, MPU_CMD cmd, uint16_t param)
{
    uint8_t data = 0;
    HAL_StatusTypeDef res = 0;

    switch (cmd) {
    case MPU_CMD_GYRO_RANGE:  /* Gyroscope full scale range */
        res = i2c_write_bits(hi2c, i2c_addr, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
        break;
    case MPU_CMD_ACCEL_RANGE: /* Accelerometer full scale range */
        res = i2c_write_bits(hi2c, i2c_addr, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
        break;
    case MPU_CMD_DLPF_CONFIG: /* Digital Low Pass Filter */
        res = i2c_write_bits(hi2c, i2c_addr, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
        break;
    case MPU_CMD_SAMPLE_RATE: /* Sample Rate = 16-bit unsigned value.
                                 Sample Rate = [1000 -  4]HZ when dlpf is enable
                                 Sample Rate = [8000 - 32]HZ when dlpf is disable */

        //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        res = i2c_read_bits(hi2c, i2c_addr, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        if (res != HAL_OK) {
            break;
        }

        if (data == 0 || data == 7) { /* dlpf is disable */
            if (param > 8000)
                data = 0;
            else if (param < 32)
                data = 0xFF;
            else
                data = 8000 / param - 1;
        } else { /* dlpf is enable */
            if (param > 1000)
                data = 0;
            else if (param < 4)
                data = 0xFF;
            else
                data = 1000 / param - 1;
        }
        res = i2c_write_reg(hi2c, i2c_addr, MPU6XXX_RA_SMPLRT_DIV, data);
        break;
    case MPU_CMD_SLEEP: /* Configure sleep mode */
        res = i2c_write_bits(hi2c, i2c_addr, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, 1, param);
        break;
    }

    return res;
}

inline void mpu_sen_init() {
  accel_sen = MPU6XXX_ACCEL_SEN >> mpu_config.mpu_accel_range;
  gyro_sen = MPU6XXX_GYRO_SEN >> mpu_config.mpu_gyro_range;
}

HAL_StatusTypeDef mpu_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *reg)
{
  HAL_StatusTypeDef res;

	res = i2c_read_regs(hi2c, i2c_addr, MPU6XXX_RA_WHO_AM_I, 1, reg); // MPU6050_WHO_AM_I; MPU6500_WHO_AM_I; MPU9250_WHO_AM_I; ...;
    if (res != HAL_OK) {
        return res;
	}

  res += i2c_write_bits(hi2c, i2c_addr, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
  res += mpu_set_param(hi2c, i2c_addr, MPU_CMD_GYRO_RANGE,  mpu_config.mpu_gyro_range);
  res += mpu_set_param(hi2c, i2c_addr, MPU_CMD_ACCEL_RANGE, mpu_config.mpu_accel_range);
  res += mpu_set_param(hi2c, i2c_addr, MPU_CMD_SLEEP,       mpu_config.mpu_sleep);
  res += mpu_set_param(hi2c, i2c_addr, MPU_CMD_DLPF_CONFIG, mpu_config.mpu_dlpf);
  res += mpu_set_param(hi2c, i2c_addr, MPU_CMD_SAMPLE_RATE, mpu_config.mpu_sample_rate);

  return res;
}
