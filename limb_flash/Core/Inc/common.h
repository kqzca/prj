/*
 * common.h
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f1xx_hal.h"

typedef struct _DATA_RAW
{
  uint16_t ad[4];
  uint8_t accel_u[6];
  uint8_t gyro_u[6];
  uint8_t accel_l[6];
  uint8_t gyro_l[6];
} DATA_RAW;

typedef struct _PAGE_RAW
{
  DATA_RAW data_raw_buffer[8];
} PAGE_RAW;

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
uint32_t wait_for_counter_changed();
void increase_250ms_counter();
uint16_t read_TIM6_counter();
uint16_t reset_TIM6_counter();

GPIO_PinState read_key0();
GPIO_PinState read_key1();
GPIO_PinState read_ext_sw();
uint8_t ext_key_start();
void write_LED0(GPIO_PinState state);
void write_LED1(GPIO_PinState state);
void write_LEDExt(GPIO_PinState state);
void LEDExt_on();
void LEDExt_off();

void srat_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_ADC(ADC_HandleTypeDef *hadc);

#endif /* INC_COMMON_H_ */
