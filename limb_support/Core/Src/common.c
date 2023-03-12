/*
 * common.c
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#include "stm32f1xx_hal.h"
#include "common.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

STATE state = INIT;
STATE get_state() {
  return state;
}
void set_state(STATE _state) {
  state = _state;
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
    write_LED1(GPIO_PIN_SET);
  }
  oldCounterValue = counterValue;
  write_LED1(GPIO_PIN_RESET);
  return counterValue;
}

inline GPIO_PinState read_key0() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4); }
inline GPIO_PinState read_key1() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3); }
inline GPIO_PinState read_ext_sw() { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); }

inline uint8_t ext_key_start() { return read_ext_sw() == GPIO_PIN_SET ? 1 : 0; }

inline void write_LED0(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state); }
inline void write_LED1(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, state); }
inline void write_LEDExt(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, state); }

void increase_250ms_counter() {
  static uint32_t counter_250ms = 0;
  ++counter_250ms;
  switch(state) {
  case INIT:
    write_LEDExt(((counter_250ms & 0x04) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case IDLE:
    write_LEDExt(GPIO_PIN_SET);
    break;
  case COLLECTING_DATA:
    write_LEDExt(((counter_250ms & 0x02) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case INIT_SD_CARD:
  case UNKNOWN:
  default:
    write_LEDExt(((counter_250ms & 0x01) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  }
}

void srat_ADC(ADC_HandleTypeDef *hadc, uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	HAL_ADC_ConfigChannel(hadc, &sConfig);
	HAL_ADC_Start(hadc);
}

inline uint16_t read_ADC(ADC_HandleTypeDef *hadc) {
	HAL_ADC_PollForConversion(hadc, 1);
	return HAL_ADC_GetValue(hadc);
}
