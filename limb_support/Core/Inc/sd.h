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

void wait_for_sd_card_state(SD_HandleTypeDef *hsd, HAL_SD_CardStateTypeDef expected_state);
void wait_for_sdio_state(SD_HandleTypeDef *hsd, HAL_SD_StateTypeDef expected_state);
void sd_page_print(SD_PAGE *page, uint8_t record_index, char *info, uint8_t rtc_second);

#endif /* INC_SD_H_ */
