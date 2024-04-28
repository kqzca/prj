/*
 * i2c.c
 *
 *  Created on: Apr 28, 2024
 *      Author: zhang
 */

#include "i2c.h"

inline HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data) {
  return HAL_I2C_Mem_Write(hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1);
}

inline HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buffer) {
  return HAL_I2C_Mem_Read(hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 1);
}

HAL_StatusTypeDef i2c_write_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t data)
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

HAL_StatusTypeDef i2c_read_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t *data)
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
