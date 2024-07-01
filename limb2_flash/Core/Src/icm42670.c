/*
 * i2c.c
 *
 *  Created on: Jun 30, 2024
 *      Author: zhang
 */

#include "icm42670.h"
#include "i2c.h"

HAL_StatusTypeDef icm42670Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
  HAL_StatusTypeDef res = icm42670Begin(hi2c, i2c_addr);
  if (res != HAL_OK) return res;
  res = icm42670SensorConf(hi2c, i2c_addr);
  if (res != HAL_OK) return res;
  res = icm42670StartAccel(hi2c, i2c_addr, ICM42670_CONFIG_ACCEL_2_G, ICM42670_CONFIG_RATE_1p6_kHz);
  if (res != HAL_OK) return res;
  res = icm42670StartGyro(hi2c, i2c_addr, ICM42670_CONFIG_GYRO_250_DPS, ICM42670_CONFIG_RATE_1p6_kHz);
  return res;
}

HAL_StatusTypeDef icm42670Begin(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
  uint8_t whoAmI;
  HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, WHO_AM_I_REG, 1, &whoAmI);
  if (res == HAL_OK) {
    if (whoAmI == ICM42670_WHO_AM_I) {
      return HAL_OK;
    } else {
      return HAL_ERROR;
    }
  } else {
    return res;
  }
}

HAL_StatusTypeDef icm42670SensorConf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
  uint8_t sensorConf = 0b00001111;
  return i2c_write_reg(hi2c, i2c_addr, ICM42670_REG_PWR_MGMT0, sensorConf);
}

HAL_StatusTypeDef icm42670StartAccel(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t scale, uint8_t freq) {
  uint16_t accelCalib = 0;
  switch (scale) {
    case ICM42670_CONFIG_ACCEL_16_G:
      accelCalib = 2048;
      break;
    case ICM42670_CONFIG_ACCEL_8_G:
      accelCalib = 4096;
      break;
    case ICM42670_CONFIG_ACCEL_4_G:
      accelCalib = 8192;
      break;
    case ICM42670_CONFIG_ACCEL_2_G:
    default:
      accelCalib = 16384;
      break;
    }
    uint8_t accelConf = scale | freq;
    // check if these settings are already stored
    uint8_t accelConfOld;
    HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, ICM42670_REG_ACCEL_CONFIG0, 1, &accelConfOld);
    if (res == HAL_OK) {
      if (accelConfOld == accelConf) {
        return HAL_OK;
      } else {
        return i2c_write_reg(hi2c, i2c_addr, ICM42670_REG_ACCEL_CONFIG0, accelConf);
      }
    }
    return res;
}

HAL_StatusTypeDef icm42670StartGyro(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t rate, uint8_t freq) {
  float gyroCalib = 0;
  switch (rate) {
    case ICM42670_CONFIG_GYRO_2k_DPS:
        gyroCalib = 16.4;
        break;
    case ICM42670_CONFIG_GYRO_1k_DPS:
        gyroCalib = 32.8;
        break;
    case ICM42670_CONFIG_GYRO_500_DPS:
        gyroCalib = 65.5;
        break;
    case ICM42670_CONFIG_GYRO_250_DPS:
    default:
        gyroCalib = 131;
        break;
    }
    uint8_t gyroConf = rate | freq;
    // check if theses settings are alreay stored
    uint8_t gyroConfOld;
    HAL_StatusTypeDef res = i2c_read_regs(hi2c, i2c_addr, ICM42670_REG_GYRO_CONFIG0, 1, &gyroConfOld);
    if (res == HAL_OK) {
      if (gyroConfOld == gyroConf) {
        return HAL_OK;
      } else {
        return i2c_write_reg(hi2c, i2c_addr, ICM42670_REG_GYRO_CONFIG0, gyroConf);
      }
    }
    // gyro needs a few millis to reconfigure
    HAL_Delay(20);
    return res;
}

void mpu_raw_to_xyz(uint8_t *buf_6_bytes, XYZ_INT16T *xyz) {
  xyz->x = ((uint16_t)buf_6_bytes[0] << 8) + buf_6_bytes[1];
  xyz->y = ((uint16_t)buf_6_bytes[2] << 8) + buf_6_bytes[3];
  xyz->z = ((uint16_t)buf_6_bytes[4] << 8) + buf_6_bytes[5];
}
