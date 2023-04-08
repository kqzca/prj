/*
 * w25q16.c
 *
 *  Created on: Mar. 26, 2023
 *      Author: zhang
 */

#include "stm32f1xx_hal.h"
#include "w25q16.h"
#include <string.h>

static const uint8_t DUMMY_BYTE = 0xA5;

void w25q16_chip_select() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void w25q16_chip_deselect() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void w25q16_init(SPI_HandleTypeDef *hspi, W25Q16_T *w25q16_info) {
  static const uint8_t CMD_READ_ID = 0x9F;
  uint8_t mf = 0, id1 = 0, id0 = 0;

  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_READ_ID, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &mf, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &id1, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &id0, 1, 1);
  w25q16_chip_deselect();

  if (id0 == 0x15) { // w25q16
    w25q16_info->PageSize = W25Q16_PAGE_SIZE;
    w25q16_info->SectorSize = 0x1000;
    w25q16_info->BlockSize = w25q16_info->SectorSize * 16;
    w25q16_info->BlockCount = 32;
    w25q16_info->SectorCount = w25q16_info->BlockCount * 16;
    w25q16_info->PageCount = (w25q16_info->SectorCount * w25q16_info->SectorSize) / w25q16_info->PageSize;
  } else {
    memset(w25q16_info, 0, sizeof(W25Q16_T));
  }
}

void w25q16_write_enable(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_WRITE_ENABLE = 0x06;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &CMD_WRITE_ENABLE, 1, 1); // Write Enable Command
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

uint8_t w25q16_read_status(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_READ_STATUS = 0x05;
  uint8_t reg_status = 0;
  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_READ_STATUS, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &reg_status, 1, 1);
  w25q16_chip_deselect();
  return reg_status;
}

void w25q16_wait_write_done(SPI_HandleTypeDef *hspi) {
  uint8_t reg_status = 0;
  uint32_t count = 0;
  do {
    ++count;
  } while ((w25q16_read_status(hspi) & 0x01) == 0x01);
}

void w25q16_erase_chip(SPI_HandleTypeDef *hspi)
{
  static const uint8_t CMD_ERASE_CHIP = 0xC7;

  w25q16_write_enable(hspi);
  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_ERASE_CHIP, 1, 1);
  w25q16_chip_deselect();

  uint8_t reg_status = 0;
  do {
    HAL_Delay(1);
  } while ((w25q16_read_status(hspi) & 0x01) == 0x01);
}

void w25q16_read(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len) {
  static const uint8_t CMD_READ = 0x03;
  uint8_t to_transmit[4];
  to_transmit[0] = CMD_READ;
  to_transmit[1] = (addr & 0x00FF0000) >> 16;
  to_transmit[2] = (addr & 0x0000FF00) >> 8;
  to_transmit[3] = (addr & 0x000000FF);
  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &to_transmit, 4, 1);
  HAL_SPI_Receive(hspi, buf, len, 1);
  w25q16_chip_deselect();
}

void w25q16_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len) {
  static const uint8_t CMD_WRITE = 0x02;
  uint8_t to_transmit[4];
  to_transmit[0] = CMD_WRITE;
  to_transmit[1] = (addr & 0x00FF0000) >> 16;
  to_transmit[2] = (addr & 0x0000FF00) >> 8;
  to_transmit[3] = (addr & 0x000000FF);
  w25q16_write_enable(hspi);
  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &to_transmit, 4, 1);
  HAL_SPI_Transmit(hspi ,buf, len, 1);
  w25q16_chip_deselect();
}
