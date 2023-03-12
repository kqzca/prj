/*
 * common.h
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "stm32f1xx_hal.h"

typedef struct _SEPARATED
{
    char index[14];       // 14 bytes, 13 bytes for index, 1 byte for comma
    char data[112];       // 16 signed integer with comma (16 bit), "+/-32767,", 112 bytes
    char line_end[2];  // last 2 bytes for "\r\n"
} SEPARATED;

typedef union _DATA_RECORD
{
    char buffer[128];
    SEPARATED separated;
} DATA_RECORD;

typedef struct _SD_PAGE
{
	DATA_RECORD data_record_buffer[4];
} SD_PAGE;

typedef struct _DATA_RAW
{
  uint16_t index;
  uint16_t ad[4];
  uint8_t accel_u[6];
  uint8_t gyro_u[6];
  uint8_t accel_l[6];
  uint8_t gyro_l[6];
} DATA_RAW;

typedef struct _SD_PAGE_RAW
{
  DATA_RAW data_raw_buffer[15];
  uint16_t not_used;
} SD_PAGE_RAW;

void wait_for_sd_card_state(SD_HandleTypeDef *hsd, HAL_SD_CardStateTypeDef expected_state);
void wait_for_sdio_state(SD_HandleTypeDef *hsd, HAL_SD_StateTypeDef expected_state);
void sd_page_print(SD_PAGE *page, uint8_t record_index, char *info, uint8_t rtc_second);

#endif /* INC_SD_H_ */
