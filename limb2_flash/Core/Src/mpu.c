/*
 * common.c
 *
 *  Created on: Feb. 20, 2023
 *      Author: Qun Zhang
 */

#include "stm32f4xx_hal.h"
#include "mpu.h"
#include "i2c.h"

static const MPU_CONFIG mpu_config = {
  MPU6XXX_ACCEL_RANGE_2G,
  MPU6XXX_GYRO_RANGE_250DPS,
  MPU6XXX_DLPF_DISABLE,
  MPU6XXX_SLEEP_DISABLE,
  1000
};

void mpu_raw_to_xyz(uint8_t *buf_6_bytes, XYZ_INT16T *xyz) {
  xyz->x = ((uint16_t)buf_6_bytes[0] << 8) + buf_6_bytes[1];
  xyz->y = ((uint16_t)buf_6_bytes[2] << 8) + buf_6_bytes[3];
  xyz->z = ((uint16_t)buf_6_bytes[4] << 8) + buf_6_bytes[5];
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
