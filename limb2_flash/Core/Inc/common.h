/*
 * common.h
 *
 *  Created on: Feb. 16, 2023
 *      Author: Qun Zhang
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f4xx_hal.h"

typedef struct _DATA_RAW
{
  uint16_t ad[6];
  uint8_t accel_u[6];
  uint8_t gyro_u[6];
  uint8_t accel_l[6];
  uint8_t gyro_l[6];
} DATA_RAW;

#define RECORD_PER_PAGE 7
typedef struct _PAGE_RAW
{
  DATA_RAW data_raw_buffer[RECORD_PER_PAGE];
} PAGE_RAW;

typedef enum _STATE
{
    NOT_READY = 0,
    READY_IDLE,
    COLLECTING_DATA,
    SAVEING_TO_FILE,
} STATE;
STATE check_state();
void set_state(STATE _state);

uint32_t get_counter();
void increase_counter();
uint32_t wait_for_counter_changed();
void increase_250ms_counter();
uint16_t read_TIM7_counter();
uint16_t reset_TIM7_counter();

GPIO_PinState read_ext_sw();
uint8_t ext_key_start();
void write_LEDGRN(GPIO_PinState state);
void write_LEDRED(GPIO_PinState state);

void start_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_ADC(ADC_HandleTypeDef *hadc);

#define TCA9543_ADDRESS_AD012_LOW     0x20
#define TCA9543_INPUT_REG_ADDR        0x00
#define I2C_DI_EXPANDER_ADDR_SHIFTED  (TCA9543_ADDRESS_AD012_LOW << 1)
uint8_t is_io_expander_input_change();
void notify_io_expander_input_change();

#endif /* INC_COMMON_H_ */
