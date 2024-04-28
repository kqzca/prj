/*
 * w25q128.h
 *
 *  Created on: Apr. 22, 2024
 *      Author: zhang
 */

#ifndef INC_W25Q128_H_
#define INC_W25Q128_H_

#include "stm32f4xx_hal.h"

#define W25Q128_PAGE_SIZE 256
#define PAGE_ADDRESS_SHIFT 8
typedef struct _W25Q128_T
{
  uint8_t UniqID[8];
  uint16_t PageSize;
  uint32_t PageCount;
  uint32_t SectorSize;
  uint32_t SectorCount;
  uint32_t BlockSize;
  uint32_t BlockCount;
} W25Q128_T;

void w25q128_chip_select();
void w25q128_chip_deselect();
void w25q128_init(SPI_HandleTypeDef *hspi, W25Q128_T *w25q128_info);
void w25q128_wait_write_done(SPI_HandleTypeDef *hspi);
uint8_t w25q128_read_status(SPI_HandleTypeDef *hspi);
void w25q128_write_enable(SPI_HandleTypeDef *hspi);
void w25q128_erase_chip(SPI_HandleTypeDef *hspi);
void w25q128_read(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
void w25q128_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
#endif // INC_W25Q128_H_
