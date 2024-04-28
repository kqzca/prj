/*
 * w25q128.c
 *
 *  Created on: Mar. 26, 2023
 *      Author: Qun Zhang
 */

#include "w25q128.h"
#include <string.h>

static const uint8_t DUMMY_BYTE = 0xA5;

void w25q128_chip_select() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void w25q128_chip_deselect() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void w25q128_init(SPI_HandleTypeDef *hspi, W25Q128_T *w25q128_info) {
  static const uint8_t CMD_READ_ID = 0x9F;
  uint8_t mf = 0, id1 = 0, id0 = 0;

  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_READ_ID, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &mf, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &id1, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &id0, 1, 1);
  w25q128_chip_deselect();

  if (id0 == 0x18) { // w25q128
    w25q128_info->PageSize = W25Q128_PAGE_SIZE;                 // 256B
    w25q128_info->SectorSize = 0x1000;                          // 4K
    w25q128_info->BlockSize = w25q128_info->SectorSize * 16;    // 64K
    w25q128_info->BlockCount = 256;                             // 256 blocks
    w25q128_info->SectorCount = w25q128_info->BlockCount * 16;  // 4096 sectors, 65536 pages
    w25q128_info->PageCount = (w25q128_info->SectorCount * w25q128_info->SectorSize) / w25q128_info->PageSize;
  } else {
    memset(w25q128_info, 0, sizeof(W25Q128_T));
  }
}

void w25q128_write_enable(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_WRITE_ENABLE = 0x06;
  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_WRITE_ENABLE, 1, 1); // Write Enable Command
  w25q128_chip_deselect();
}

uint8_t w25q128_read_status(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_READ_STATUS = 0x05;
  uint8_t reg_status = 0;
  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_READ_STATUS, 1, 1);
  HAL_SPI_TransmitReceive(hspi, &DUMMY_BYTE, &reg_status, 1, 1);
  w25q128_chip_deselect();
  return reg_status;
}

void w25q128_wait_write_done(SPI_HandleTypeDef *hspi) {
  uint32_t count = 0;
  do {
    ++count;
  } while ((w25q128_read_status(hspi) & 0x01) == 0x01);
}

void w25q128_erase_chip(SPI_HandleTypeDef *hspi)
{
  static const uint8_t CMD_ERASE_CHIP = 0xC7;

  w25q128_write_enable(hspi);
  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_ERASE_CHIP, 1, 1);
  w25q128_chip_deselect();

  do {
    HAL_Delay(1);
  } while ((w25q128_read_status(hspi) & 0x01) == 0x01);
}

void w25q128_read(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len) {
  static const uint8_t CMD_READ = 0x03;
  uint8_t to_transmit[4];
  to_transmit[0] = CMD_READ;
  to_transmit[1] = (addr & 0x00FF0000) >> 16;
  to_transmit[2] = (addr & 0x0000FF00) >> 8;
  to_transmit[3] = (addr & 0x000000FF);
  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, to_transmit, 4, 1);
  HAL_SPI_TransmitReceive(hspi, buf, buf, len, 2);
  w25q128_chip_deselect();
}

void w25q128_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len) {
  static const uint8_t CMD_WRITE = 0x02;
  uint8_t to_transmit[4];
  to_transmit[0] = CMD_WRITE;
  to_transmit[1] = (addr & 0x00FF0000) >> 16;
  to_transmit[2] = (addr & 0x0000FF00) >> 8;
  to_transmit[3] = (addr & 0x000000FF);
  w25q128_write_enable(hspi);
  w25q128_chip_select();
  HAL_SPI_Transmit(hspi, to_transmit, 4, 1);
  HAL_SPI_Transmit(hspi, buf, len, 2);
  w25q128_chip_deselect();
}
