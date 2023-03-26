/*
 * common.h
 *
 *  Created on: Mar. 26, 2023
 *      Author: zhang
 */

#ifndef INC_W25Q16_H_
#define INC_W25Q16_H_

#include "stm32f1xx_hal.h"

void w25q16_chip_select();
void w25q16_chip_deselect();
void w25q16_wait_write_done(SPI_HandleTypeDef *hspi);
uint8_t w25q16_read_status(SPI_HandleTypeDef *hspi);
void w25q16_write_enable(SPI_HandleTypeDef *hspi);
void w25q16_erase_chip(SPI_HandleTypeDef *hspi);
void w25q16_read(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
void w25q16_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
#endif // INC_W25Q16_H_
