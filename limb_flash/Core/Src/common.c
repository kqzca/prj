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
    write_LED1(GPIO_PIN_RESET);
  }
  oldCounterValue = counterValue;
  write_LED1(GPIO_PIN_SET);
  return counterValue;
}

inline GPIO_PinState read_key0() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4); }
inline GPIO_PinState read_key1() { return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3); }
inline GPIO_PinState read_ext_sw() { return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8); }

inline uint8_t ext_key_start() { return read_ext_sw() == GPIO_PIN_SET ? 1 : 0; }

inline void write_LED0(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state); }
inline void write_LED1(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, state); }
inline void write_LEDExt(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, state); }

void increase_250ms_counter() {
  static uint32_t counter_250ms = 0;
  ++counter_250ms;
  switch(running_state) {
  case READY_IDLE:
    write_LEDExt(GPIO_PIN_SET);
    break;
  case COLLECTING_DATA:
    write_LEDExt(((counter_250ms & 0x02) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case SAVEING_TO_FILE:
    write_LEDExt(((counter_250ms & 0x04) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case NOT_READY:
  default:
    write_LEDExt(((counter_250ms & 0x01) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  }
}

inline uint16_t read_TIM6_counter() {
  return TIM6->CNT;
}

inline uint16_t reset_TIM6_counter() {
  return TIM6->CNT = 0;
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
