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
    MOTOR_TEST,
} STATE;
STATE check_state();
void set_state(STATE _state);

void increase_2ms_counter();
uint32_t wait_for_counter_changed();
void increase_250ms_counter();

typedef enum _INPUT_SW {
  ON_BOARD_SW,
  ROTARY_BOARD_SW
} INPUT_SW;
GPIO_PinState read_input_sw(INPUT_SW input_sw);
uint8_t input_sw_on(INPUT_SW input_sw);
typedef enum _MOTOR_CMD {
  NO_CMD,
  START,
  STOP
} MOTOR_CMD;
typedef enum _MOTOR_STATE {
  COAST,
  REVERSE,
  FORWARD,
  HOLD
} MOTOR_STATE;
MOTOR_CMD read_motor_cmd();
uint16_t cal_pwm_timer_counter(uint8_t spd_ctrl);
uint16_t decide_motor_state(uint8_t state_ctrl, uint8_t circle_target_1, uint8_t circle_target_2);
void motor_ctrl_str(uint16_t output);
void motor_ctrl_ben(uint16_t output);
void motor_stop();
void generate_pwm();

uint16_t read_TIM14_counter();
uint16_t reset_TIM14_counter();

void write_LEDGRN(GPIO_PinState state);
void write_LEDRED(GPIO_PinState state);
void toggle_LEDGRN();
void toggle_LEDRED();
void vm_en(GPIO_PinState state);

void start_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_ADC(ADC_HandleTypeDef *hadc);

#define TCA9534_ADDRESS_ADR000        0x20
#define I2C_DI_EXPANDER_ADDR_SHIFTED  (TCA9534_ADDRESS_ADR000 << 1)
#define TCA9534_INPUT_REG_ADDR 0x00
HAL_StatusTypeDef tca9534Read(I2C_HandleTypeDef *hi2c, uint8_t *buf);
uint8_t is_io_expander_input_change();
void notify_io_expander_input_change();

#define TCA9535_ADDRESS_ADR100        0x24
#define I2C_CTRL_BOARD_ADDR_SHIFTED   (TCA9535_ADDRESS_ADR100 << 1)
#define TCA9535_INPUT_REG0_ADDR 0x00
#define TCA9535_INPUT_REG1_ADDR 0x01
HAL_StatusTypeDef tca9535Read(I2C_HandleTypeDef *hi2c, uint8_t *buf);

#endif /* INC_COMMON_H_ */
