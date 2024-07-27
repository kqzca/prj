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
static uint8_t on_board_sw_st_old = 0;
static uint8_t rotary_board_sw_st_old = 0;
STATE check_state() {
  uint8_t on_board_sw_st_new = input_sw_on(ON_BOARD_SW);
  switch(running_state) {
  case READY_IDLE:
    if ((on_board_sw_st_old == 0) && (on_board_sw_st_new == 1)) {
      set_state(COLLECTING_DATA);
    }
    break;
  case COLLECTING_DATA:
    if (on_board_sw_st_new == 0) {
      set_state(SAVEING_TO_FILE);
    }
    break;
  case SAVEING_TO_FILE:
  case NOT_READY:
  case MOTOR_TEST:
  default:
    break;
  }
  on_board_sw_st_old = on_board_sw_st_new;
  return running_state;
}
void set_state(STATE _state) {
  running_state = _state;
  on_board_sw_st_old = input_sw_on(ON_BOARD_SW);
  rotary_board_sw_st_old = input_sw_on(ROTARY_BOARD_SW);
}
MOTOR_CMD read_motor_cmd() {
  MOTOR_CMD rtn = NO_CMD;
  uint8_t rotary_board_sw_st_new = input_sw_on(ROTARY_BOARD_SW);
  if ((rotary_board_sw_st_old == 0) && (rotary_board_sw_st_new == 1)) {
    rtn = START;
  } else if ((rotary_board_sw_st_old == 1) && (rotary_board_sw_st_new == 0)) {
    rtn = STOP;
  }
  rotary_board_sw_st_old = rotary_board_sw_st_new;
  return rtn;
}
uint16_t cal_pwm_timer_counter(uint8_t spd_ctrl) {
  switch (spd_ctrl) {
  case 1:   return 63000; // 100RPM
  case 2:   return 42000; // 150RPM
  case 3:   return 21000; // 300RPM
  case 4:   return 10500; // 600RPM
  case 5:   return 6300;  // 1000RPM
  case 6:   return 4200;  // 1500RPM
  case 7:   return 2100;  // 3000RPM
  case 8:   return 1050;  // 6000RPM
  case 9:   return 700;   // 9000RPM
  case 0:
  default:  return 0;
  }
}
static MOTOR_STATE motor_state_str = COAST;
static MOTOR_STATE motor_state_ben = COAST;
static uint8_t motor_str_circle_tartget = 0;
static uint8_t motor_ben_circle_tartget = 0;
uint16_t decide_motor_state(uint8_t state_ctrl, uint8_t circle_target_1, uint8_t circle_target_2) {
  motor_state_str = COAST;
  motor_state_ben = COAST;
  motor_str_circle_tartget = 0;
  motor_ben_circle_tartget = 0;
  uint8_t circle_target = circle_target_1 * 10 + circle_target_2;
  switch (state_ctrl) {
  case 1:
    motor_state_str = FORWARD;
    motor_str_circle_tartget = circle_target;
    break;
  case 2:
    motor_state_str = REVERSE;
    motor_str_circle_tartget = circle_target;
    break;
  case 3:
    motor_state_ben = FORWARD;
    motor_ben_circle_tartget = circle_target;
    break;
  case 4:
    motor_state_ben = REVERSE;
    motor_ben_circle_tartget = circle_target;
    break;
  case 9:
    motor_state_ben = BRAKE;
    motor_state_ben = BRAKE;
    break;
  case 0:
  default:
    break;
  }
}
static const uint16_t steps_output[4] = {0b0101, 0b0110, 0b1010, 0b1001};
static int8_t motor_str_forward_step_index = -1;
static int8_t motor_ben_forward_step_index = -1;
static int8_t motor_str_reverse_step_index = 4;
static int8_t motor_ben_reverse_step_index = 4;
static uint8_t motor_str_step_count = 0;
static uint8_t motor_ben_step_count = 0;
static uint8_t motor_str_circle_count = 0;
static uint8_t motor_ben_circle_count = 0;
inline void motor_ctrl_str(uint16_t output) {
  uint16_t output_negtive = ~output & 0x0f;            // RESET
  uint32_t bsrr = (uint32_t)output_negtive << 16U | output; // PA3-0
  GPIOA->BSRR = bsrr;
}
inline void motor_ctrl_ben(uint16_t output) {
  uint16_t output_negtive = ~output & 0x0f;            // RESET
  uint32_t bsrr = (uint32_t)output_negtive << 22U | output << 6U; // PB9-6
  GPIOB->BSRR = bsrr;
}
void motor_stop() {
  motor_state_str = BRAKE;
  motor_state_ben = BRAKE;
  motor_str_circle_tartget = 0;
  motor_ben_circle_tartget = 0;
  motor_str_forward_step_index = -1;
  motor_ben_forward_step_index = -1;
  motor_str_reverse_step_index = 4;
  motor_ben_reverse_step_index = 4;
  motor_str_step_count = 0;
  motor_ben_step_count = 0;
  motor_str_circle_count = 0;
  motor_ben_circle_count = 0;
  write_LEDGRN(GPIO_PIN_RESET);
  write_LEDRED(GPIO_PIN_RESET);
  motor_ctrl_str(0x0F);
  motor_ctrl_ben(0x0F);
}
void generate_pwm() {
  if (motor_state_str == FORWARD) {
    motor_str_forward_step_index++;
    if (motor_str_forward_step_index > 3) {
      motor_str_forward_step_index = 0;
      motor_str_step_count++;
    }
    uint16_t output = steps_output[motor_str_forward_step_index];
    motor_ctrl_str(output);
  } else if (motor_state_str == REVERSE) {
    motor_str_reverse_step_index--;
    if (motor_str_reverse_step_index < 0) {
      motor_str_reverse_step_index = 3;
      motor_str_step_count++;
    }
    uint16_t output = steps_output[motor_str_reverse_step_index];
    motor_ctrl_str(output);
  }
  if (motor_str_step_count == 200) {
    motor_str_step_count = 0;
    motor_str_circle_count++;
    toggle_LEDRED();
    if (motor_str_circle_count == motor_str_circle_tartget) {
      motor_stop();
    }
  }
  if (motor_state_ben == FORWARD) {
    motor_ben_forward_step_index++;
    if (motor_ben_forward_step_index > 3) {
      motor_ben_forward_step_index = 0;
      motor_ben_step_count++;
    }
    uint16_t output = steps_output[motor_ben_forward_step_index];
    motor_ctrl_ben(output);
  } else if (motor_state_ben == REVERSE) {
    motor_ben_reverse_step_index--;
    if (motor_ben_reverse_step_index < 0) {
      motor_ben_reverse_step_index = 3;
      motor_ben_step_count++;
    }
    uint16_t output = steps_output[motor_ben_reverse_step_index]; // SET
    motor_ctrl_ben(output);
  }
  if (motor_ben_step_count == 200) {
    motor_ben_step_count = 0;
    motor_ben_circle_count++;
    toggle_LEDRED();
    if (motor_ben_circle_count == motor_ben_circle_tartget) {
      motor_stop();
    }
  }
}

inline uint16_t read_TIM14_counter() {
  return TIM14->CNT;
}

inline uint16_t reset_TIM14_counter() {
  return TIM14->CNT = 0;
}

uint32_t counterValue2ms = 0, oldcounterValue2ms = 0;
void increase_2ms_counter() {
  counterValue2ms++;
}
uint32_t wait_for_counter_changed() {
  while(oldcounterValue2ms == counterValue2ms) {
    write_LEDRED(GPIO_PIN_SET);
  }
  oldcounterValue2ms = counterValue2ms;
  write_LEDRED(GPIO_PIN_RESET);
  return counterValue2ms;
}

inline GPIO_PinState read_input_sw(INPUT_SW input_sw) {
  switch (input_sw) {
  case ON_BOARD_SW:
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
  case ROTARY_BOARD_SW:
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
  default:
    return GPIO_PIN_RESET;
  }
}

inline uint8_t input_sw_on(INPUT_SW input_sw) { return read_input_sw(input_sw) == GPIO_PIN_SET ? 1 : 0; }

inline void write_LEDGRN(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, state); }
inline void write_LEDRED(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, state); }
inline void toggle_LEDGRN() { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); }
inline void toggle_LEDRED() { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); }
inline void vm_en(GPIO_PinState state) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, state); };

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
  case NOT_READY:
    write_LEDRED(GPIO_PIN_RESET);
    write_LEDGRN(((counter_250ms & 0x01) != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case MOTOR_TEST:
  default:
    break;
  }
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
