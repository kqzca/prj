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

const size_t MAX_DATA_RECORD_LENGTH = 126;
const char LINE_ENDING[2] = "\r\n";
static inline void add_lind_end(DATA_RECORD *_data_record_buffer) {
	memcpy(_data_record_buffer->separated.line_end, LINE_ENDING, sizeof(LINE_ENDING));
}

void sd_page_print(SD_PAGE *page, uint8_t record_index, char *info, uint8_t rtc_second) {
  if ((page == 0) || (record_index > 3)) {
    return;
  }
  add_lind_end(&page->data_record_buffer[record_index]);
  int32_t len = snprintf(page->data_record_buffer[record_index].buffer, MAX_DATA_RECORD_LENGTH, "%02d,%10ld,%s", rtc_second, get_counter(), info);
  memset(page->data_record_buffer[record_index].buffer + len, ' ', 126 - len);
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
