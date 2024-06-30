/*
 * i2c.h
 *
 *  Created on: Jun 30, 2024
 *      Author: zhang
 */

#ifndef INC_ICM42670_H
#define INC_ICM42670_H

#include "stm32f4xx_hal.h"

// I2C Address
#define ICM42670L_DEFAULT_ADDRESS   (0x68)
#define ICM42670U_DEFAULT_ADDRESS   (0x69)
#define ICM42670L_DEFAULT_ADDRESS_SHIFTED  (ICM42670L_DEFAULT_ADDRESS << 1)
#define ICM42670U_DEFAULT_ADDRESS_SHIFTED  (ICM42670U_DEFAULT_ADDRESS << 1)
#define ICM42670_WHO_AM_I  (0x67)
// Registers
// Data
#define ICM42670_REG_TEMP_DATA1     (0x09)
#define ICM42670_REG_TEMP_DATA0     (0x0A)
#define ICM42670_REG_ACCEL_DATA_X1  (0x0B)
#define ICM42670_REG_ACCEL_DATA_X0  (0x0C)
#define ICM42670_REG_ACCEL_DATA_Y1  (0x0D)
#define ICM42670_REG_ACCEL_DATA_Y0  (0x0E)
#define ICM42670_REG_ACCEL_DATA_Z1  (0x0F)
#define ICM42670_REG_ACCEL_DATA_Z0  (0x10)
#define ICM42670_REG_GYRO_DATA_X1   (0x11)
#define ICM42670_REG_GYRO_DATA_X0   (0x12)
#define ICM42670_REG_GYRO_DATA_Y1   (0x13)
#define ICM42670_REG_GYRO_DATA_Y0   (0x14)
#define ICM42670_REG_GYRO_DATA_Z1   (0x15)
#define ICM42670_REG_GYRO_DATA_Z0   (0x16)
//Config
#define ICM42670_REG_DRIVE_CONFIG2  (0x04)
#define ICM42670_REG_GYRO_CONFIG0   (0x20)
#define ICM42670_REG_GYRO_CONFIG1   (0x23)
#define ICM42670_REG_ACCEL_CONFIG0  (0x21)
#define ICM42670_REG_ACCEL_CONFIG1  (0x24)
#define ICM42670_REG_TEMP_CONFIG0   (0x22)
#define WHO_AM_I_REG                (0x75)
#define ICM42670_REG_PWR_MGMT0      (0x1F)

// Calibration
// GYRO
#define ICM42670_CONFIG_GYRO_2k_DPS     (0b00000000)
#define ICM42670_CONFIG_GYRO_1k_DPS     (0b00100000)
#define ICM42670_CONFIG_GYRO_500_DPS    (0b01000000)
#define ICM42670_CONFIG_GYRO_250_DPS    (0b01100000)
// ACCEL
#define ICM42670_CONFIG_ACCEL_16_G      (0b00000000)
#define ICM42670_CONFIG_ACCEL_8_G       (0b00100000)
#define ICM42670_CONFIG_ACCEL_4_G       (0b01000000)
#define ICM42670_CONFIG_ACCEL_2_G       (0b01100000)
// RATE
#define ICM42670_CONFIG_RATE_1p6_kHz    (0b00000101)
#define ICM42670_CONFIG_RATE_800_Hz     (0b00000110)
#define ICM42670_CONFIG_RATE_400_Hz     (0b00000111)
#define ICM42670_CONFIG_RATE_200_Hz     (0b00001000)
#define ICM42670_CONFIG_RATE_100_Hz     (0b00001001)
#define ICM42670_CONFIG_RATE_50_Hz      (0b00001010)
#define ICM42670_CONFIG_RATE_25_Hz      (0b00001011)
#define ICM42670_CONFIG_RATE_12p5_Hz    (0b00001100)

typedef struct _XYZ_INT16T
{
  int16_t x;
  int16_t y;
  int16_t z;
} XYZ_INT16T;

HAL_StatusTypeDef icm42670Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef icm42670Begin(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef icm42670SensorConf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef icm42670StartAccel(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t scale, uint8_t freq);
HAL_StatusTypeDef icm42670StartGyro(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t rate, uint8_t freq);
void mpu_raw_to_xyz(uint8_t *buf_6_bytes, XYZ_INT16T *xyz);

#endif /* INC_ICM42670_H */
