/*
 * w25q16.c
 *
 *  Created on: Mar. 26, 2023
 *      Author: zhang
 */

#include "stm32f1xx_hal.h"
#include "w25q16.h"

void w25q16_chip_select() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void w25q16_chip_deselect() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void w25q16_write_enable(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_WRITE_ENABLE = 0x06;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &CMD_WRITE_ENABLE, 1, 1); // Write Enable Command
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

uint8_t w25q16_read_status(SPI_HandleTypeDef *hspi) {
  static const uint8_t CMD_READ_STATUS = 0x05;
  static const uint8_t DUMMY_BYTE = 0xA5;
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
  static const uint8_t CMD_FAST_READ = 0x0B;

  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &CMD_FAST_READ, 1, 1);
  uint8_t to_transmit = (addr & 0x00FF0000) >> 16;
  HAL_SPI_Transmit(hspi, &to_transmit, 1, 1);
  to_transmit = (addr & 0x0000FF00) >> 8;
  HAL_SPI_Transmit(hspi, &to_transmit, 1, 1);
  to_transmit = (addr & 0x000000FF);
  HAL_SPI_Transmit(hspi, &to_transmit, 1, 1);
  to_transmit = 0;
  HAL_SPI_Transmit(hspi, &to_transmit, 1, 1);
  HAL_SPI_Receive(hspi, buf, len, 1);
  w25q16_chip_deselect();
}

void w25q16_write(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len) {
  uint8_t to_transmit[4];
  to_transmit[0] = 0x02;
  to_transmit[1] = (addr & 0x00FF0000) >> 16;
  to_transmit[2] = (addr & 0x0000FF00) >> 8;
  to_transmit[3] = (addr & 0x000000FF);
  w25q16_write_enable(hspi);
  w25q16_chip_select();
  HAL_SPI_Transmit(hspi, &to_transmit, 4, 1);
  HAL_SPI_Transmit(hspi ,buf, len, 1);
  w25q16_chip_deselect();
}
