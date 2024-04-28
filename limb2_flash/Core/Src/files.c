/*
 * files.c
 *
 *  Created on: Apr. 2, 2023
 *      Author: Qun Zhang
 */

#include "bsp_driver_sd.h"
#include "files.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

const TCHAR *INDEX_FILE_NAME = "index.txt";

uint8_t sd_present = SD_NOT_PRESENT;
inline void set_sd_present(uint8_t is_sd_present) {
  if (is_sd_present == 0) {
    sd_present = SD_NOT_PRESENT;
  } else {
    sd_present = SD_PRESENT;
  }
}
uint8_t BSP_SD_IsDetected(void)
{
  // No SD detect pin on board
  return sd_present;
}

uint16_t get_file_index() {
  uint16_t index = 0;
  FRESULT fatfs_res;
  if(f_open(&SDFile, INDEX_FILE_NAME, FA_OPEN_EXISTING | FA_READ) == FR_OK)
  {
    uint16_t bytesread;
    uint8_t read_buf[16];
    fatfs_res = f_read(&SDFile, read_buf, _MAX_SS, (void *)&bytesread);
    f_close(&SDFile);
    if((bytesread > 0) && (fatfs_res == FR_OK)) {
      index = strtoul(read_buf, NULL, 0);
    }
  }
  ++index;
  if(f_open(&SDFile, INDEX_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
    return 0;
  } else {
    uint16_t byteswritten;
    uint8_t write_buf[16];
    snprintf(write_buf, 16, "%d\r\n", index);
    fatfs_res = f_write(&SDFile, write_buf, strlen(write_buf), (void *)&byteswritten);
    f_close(&SDFile);
    if (fatfs_res != FR_OK) {
      return 0;
    } else {
      return index;
    }
  }
}

uint16_t write_data_record(const char *buf, size_t size) {
  FRESULT fatfs_res;
  uint16_t byteswritten;
  fatfs_res = f_write(&SDFile, buf, size, (void *)&byteswritten);
  if (fatfs_res != FR_OK) {
    return 0;
  } else {
    return byteswritten;
  }
}
