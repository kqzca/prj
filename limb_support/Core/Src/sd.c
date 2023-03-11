/*
 * common.c
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#include "sd.h"
#include "common.h"
#include <string.h>
#include <stdio.h>

const size_t MAX_DATA_RECORD_LENGTH = 112;
const char PADDING_AND_LINE_ENDING[18] = "                \r\n";
static inline void pad_buf(DATA_RECORD *_data_record_buffer) {
	memcpy(_data_record_buffer->padding, PADDING_AND_LINE_ENDING, sizeof(PADDING_AND_LINE_ENDING));
}

void sd_page_print(SD_PAGE *page, uint8_t record_index, char *info) {
  if ((page == 0) || (record_index > 3)) {
    return;
  }
  pad_buf(&page->data_record_buffer[record_index]);
  snprintf(page->data_record_buffer[record_index].index, 14, "%13ld", get_counter());
  int32_t len = snprintf(page->data_record_buffer[record_index].data, MAX_DATA_RECORD_LENGTH, "%s", info);
  memset(page->data_record_buffer[record_index].data + len, ' ', 96 - len);
}

void sd_page_print_header(SD_PAGE *page, uint8_t record_index) {
  const char *TABLE_HEADER = "----index----,ana_1,ana_2,ana_3,ana_4,acc1x,acc1y,acc1z,gyro1x,gyro1y,gyro1z,acc1x,acc1y,acc1z,gyro1x,gyro1y,gyro1z,";
  pad_buf(&page->data_record_buffer[record_index]);
  size_t table_header_length = strlen(TABLE_HEADER);
  memcpy(page->data_record_buffer[record_index].index, TABLE_HEADER,
      (table_header_length <= MAX_DATA_RECORD_LENGTH) ? table_header_length : MAX_DATA_RECORD_LENGTH);
}

void wait_for_sd_card_state(SD_HandleTypeDef *hsd, HAL_SD_CardStateTypeDef expected_state) {
  HAL_SD_CardStateTypeDef sd_card_state = HAL_SD_GetCardState(hsd);
  while(sd_card_state != expected_state) {
    sd_card_state = HAL_SD_GetCardState(hsd);
    LEDExt_flash_slow();
  }
}

void wait_for_sdio_state(SD_HandleTypeDef *hsd, HAL_SD_StateTypeDef expected_state) {
  while(hsd->State != expected_state) {
    LEDExt_flash_slow();
  }
}
