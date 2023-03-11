/*
 * common.h
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f1xx_hal.h"

typedef enum _STATE
{
    INIT = 0,
    IDLE,
    COLLECTING_DATA,
    INIT_SD_CARD,
    UNKNOWN
} STATE;
STATE get_state();
void set_state(STATE _state);

uint32_t get_counter();
void increase_counter();
void wait_for_counter_changed();

GPIO_PinState read_key0();
GPIO_PinState read_key1();
GPIO_PinState read_ext_sw();
uint8_t ext_key_start();
void write_LED0(GPIO_PinState state);
void write_LED1(GPIO_PinState state);
void write_LEDExt(GPIO_PinState state);
void LEDExt_on();
void LEDExt_off();
void LEDExt_flash_slow(); // ~ 1024 ms
void LEDExt_flash();      // ~ 512 ms
void LEDExt_flash_fast(); // ~ 256 ms

void srat_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_ADC(ADC_HandleTypeDef *hadc);

#endif /* INC_COMMON_H_ */
