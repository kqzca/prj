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
inline uint32_t get_counter() { return counterValue; }
inline void sync_counter() { oldCounterValue = counterValue; }
void increase_counter() {
  counterValue++;
  switch(state) {
  case INIT:
    LEDExt_flash_slow();
    break;
  case IDLE:
    LEDExt_on();
    break;
  case COLLECTING_DATA:
    LEDExt_flash();
    break;
  case INIT_SD_CARD:
  case UNKNOWN:
  default:
    LEDExt_flash_fast();
    break;
  }
}
void wait_for_counter_changed() {
  while(oldCounterValue == counterValue) {
    write_LED1(GPIO_PIN_SET);
  }
  oldCounterValue = counterValue;
  write_LED1(GPIO_PIN_RESET);
}

inline GPIO_PinState read_key0() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4); }
inline GPIO_PinState read_key1() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3); }
inline GPIO_PinState read_ext_sw() { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); }

inline uint8_t ext_key_start() { return read_ext_sw() == GPIO_PIN_SET ? 1 : 0; }

inline void write_LED0(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state); }
inline void write_LED1(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, state); }
inline void write_LEDExt(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, state); }

inline void LEDExt_on() {
  write_LEDExt(GPIO_PIN_SET);
}
inline void LEDExt_off() {
  write_LEDExt(GPIO_PIN_RESET);
}
inline void LEDExt_flash_slow() { // ~ 1024 ms
  write_LEDExt(((get_counter() & 0x0100) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
inline void LEDExt_flash() {      // ~ 512 ms
  write_LEDExt(((get_counter() & 0x0080) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
inline void LEDExt_flash_fast() { // ~ 256 ms
  write_LEDExt(((get_counter() & 0x0040) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
