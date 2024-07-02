/*
 * common.c
 *
 *  Created on: Feb. 16, 2023
 *      Author: Qun Zhang
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "common.h"

static STATE running_state = NOT_READY;
static uint8_t ext_key_start_old = 0;
STATE check_state() {
  uint8_t ext_key_start_new = ext_key_start();
  switch(running_state) {
  case READY_IDLE:
    if ((ext_key_start_old == 0) && (ext_key_start_new == 1)) {
      set_state(COLLECTING_DATA);
    }
    break;
  case COLLECTING_DATA:
    if (ext_key_start_new == 0) {
      set_state(SAVEING_TO_FILE);
    }
    break;
  case SAVEING_TO_FILE:
  case NOT_READY:
  case MOTOR_TEST:
  default:
    break;
  }
  ext_key_start_old = ext_key_start_new;
  return running_state;
}
void set_state(STATE _state) {
  running_state = _state;
  ext_key_start_old = ext_key_start();
}

uint32_t counterValue = 0, oldCounterValue = 0;
inline uint32_t get_counter() {
  return counterValue;
}

void increase_counter() {
  counterValue++;
}

uint32_t wait_for_counter_changed() {
  while(oldCounterValue == counterValue) {
    write_LEDRED(GPIO_PIN_SET);
  }
  oldCounterValue = counterValue;
  write_LEDRED(GPIO_PIN_RESET);
  return counterValue;
}

inline GPIO_PinState read_ext_sw() { return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8); }

inline uint8_t ext_key_start() { return read_ext_sw() == GPIO_PIN_SET ? 1 : 0; }

inline void write_LEDGRN(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, state); }
inline void write_LEDRED(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, state); }

void increase_250ms_counter() {
  static uint32_t counter_250ms = 0;
  ++counter_250ms;
  switch(running_state) {
  case READY_IDLE:
    write_LEDRED(GPIO_PIN_RESET);
    write_LEDGRN(GPIO_PIN_SET);
    break;
  case COLLECTING_DATA:
    write_LEDRED(GPIO_PIN_RESET);
    write_LEDGRN(((counter_250ms & 0x02) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case SAVEING_TO_FILE:
    write_LEDRED(GPIO_PIN_RESET);
    write_LEDGRN(((counter_250ms & 0x04) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case MOTOR_TEST:
    write_LEDRED(((counter_250ms & 0x04) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    write_LEDGRN(GPIO_PIN_RESET);
    break;
  case NOT_READY:
    write_LEDRED(GPIO_PIN_RESET);
    write_LEDGRN(((counter_250ms & 0x01) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  default:
    write_LEDRED(GPIO_PIN_SET);
    write_LEDGRN(GPIO_PIN_RESET);
  }
}

inline uint16_t read_TIM7_counter() {
  return TIM7->CNT;
}

inline uint16_t reset_TIM7_counter() {
  return TIM7->CNT = 0;
}

void start_ADC(ADC_HandleTypeDef *hadc, uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  HAL_ADC_ConfigChannel(hadc, &sConfig);
  HAL_ADC_Start(hadc);
}

inline uint16_t read_ADC(ADC_HandleTypeDef *hadc) {
  HAL_ADC_PollForConversion(hadc, 1);
  return HAL_ADC_GetValue(hadc);
}

inline HAL_StatusTypeDef tca9534Read(I2C_HandleTypeDef *hi2c, uint8_t *buf) {
  return i2c_read_regs(hi2c, I2C_DI_EXPANDER_ADDR_SHIFTED, TCA9534_INPUT_REG_ADDR, 1, buf);
}
uint8_t io_expander_input_changed = 0;
inline uint8_t is_io_expander_input_change() {
  uint8_t res = io_expander_input_changed;
  io_expander_input_changed = 0;
  return res;
}
inline void notify_io_expander_input_change() {
  io_expander_input_changed = 1;
}

inline HAL_StatusTypeDef tca9535Read(I2C_HandleTypeDef *hi2c, uint8_t *buf) {
  return i2c_read_regs(hi2c, I2C_CTRL_BOARD_ADDR_SHIFTED, TCA9535_INPUT_REG0_ADDR, 2, buf);
}
inline uint8_t isCtrlBoardConnected() {
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET ? 1 : 0;
}
