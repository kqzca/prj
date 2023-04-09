/*
 * files.h
 *
 *  Created on: Apr. 2, 2023
 *      Author: zhang
 */

#ifndef INC_FILES_H_
#define INC_FILES_H_

#include "fatfs.h"

#define CVS_DATA_SIZE_MAX 128
uint16_t get_file_index();
uint16_t write_data_record(const char *buf, size_t size);

#endif /* INC_FILES_H_ */
