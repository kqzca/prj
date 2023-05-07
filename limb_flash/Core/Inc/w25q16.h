/*
 * common.h
 *
 *  Created on: Mar. 26, 2023
 *      Author: zhang
 */

#ifndef INC_W25Q16_H_
#define INC_W25Q16_H_

#include "stm32f1xx_hal.h"

#define W25Q16_PAGE_SIZE 256
#define PAGE_ADDRESS_SHIFT 8
typedef struct _W25Q16_T
{
  uint8_t UniqID[8];
  uint16_t PageSize;
  uint32_t PageCount;
  uint32_t SectorSize;
  uint32_t SectorCount;
  uint32_t BlockSize;
  uint32_t BlockCount;
} W25Q16_T;

void w25q16_chip_select();
void w25q16_chip_deselect();
void w25q16_init(SPI_HandleTypeDef *hspi, W25Q16_T *w25q16_info);
void w25q16_wait_write_done(SPI_HandleTypeDef *hspi);
uint8_t w25q16_read_status(SPI_HandleTypeDef *hspi);
void w25q16_write_enable(SPI_HandleTypeDef *hspi);
void w25q16_erase_chip(SPI_HandleTypeDef *hspi);
void w25q16_read(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
void w25q16_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
#endif // INC_W25Q16_H_
