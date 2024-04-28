/*
 * i2c.h
 *
 *  Created on: Apr 28, 2024
 *      Author: zhang
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buffer);
HAL_StatusTypeDef i2c_write_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t data);
HAL_StatusTypeDef i2c_read_bits(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t start_bit, uint8_t len, uint8_t *data);

#endif /* INC_I2C_H_ */
