/*
 * common.h
 *
 *  Created on: Feb. 26, 2023
 *      Author: zhang
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f1xx_hal.h"
#include "mpu6xxx_reg.h"

typedef enum _STATE
{
    INIT = 0,
    IDLE,
    COLLECTING_DATA,
    ERROR_NO_SD_CARD,
    UNKNOWN
} STATE;

typedef struct _DATA_RECORD
{
    char index_and_data[110];			// index 13 bytes, one comma, plus 16 signed integer with comma (16 bit), "+/-32767,", min 96 bytes, max 112 bytes
    char padding_and_line_ending[18];	// 16 bytes padding with ' ', last 2 bytes for "\r\n"
} DATA_RECORD;

typedef struct _SD_PAGE
{
	DATA_RECORD data_record_buffer[4];
} SD_PAGE;

void pad_buf(DATA_RECORD *_data_record_buffer);
void wait_for_sd_card_state(SD_HandleTypeDef *hsd, HAL_SD_CardStateTypeDef expected_state);
void wait_for_sdio_state(SD_HandleTypeDef *hsd, HAL_SD_StateTypeDef expected_state);
void sd_page_print(SD_PAGE *page, uint8_t record_index, char *info);
void sd_page_print_header(SD_PAGE *page, uint8_t record_index);

uint32_t get_counter();
void increase_counter();
void wait_for_counter_changed();

GPIO_PinState read_key0();
GPIO_PinState read_key1();
GPIO_PinState read_ext_sw();
uint8_t user_command_start();
void write_LED0(GPIO_PinState state);
void write_LED1(GPIO_PinState state);
void write_LEDExt(GPIO_PinState state);
void LEDExt_flash_slow(); // ~ 1024 ms
void LEDExt_flash(); // ~ 512 ms
void LEDExt_flash_fast(); // ~ 256 ms

void srat_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_ADC(ADC_HandleTypeDef *hadc);

/* Accelerometer full scale range */
typedef enum _MPU_ACCEL_RANGE
{
    MPU6XXX_ACCEL_RANGE_2G  = 0, // ±2G
    MPU6XXX_ACCEL_RANGE_4G  = 1, // ±4G
    MPU6XXX_ACCEL_RANGE_8G  = 2, // ±8G
    MPU6XXX_ACCEL_RANGE_16G = 3  // ±16G
} MPU_ACCEL_RANGE;

/* Gyroscope full scale range */
typedef enum _MPU_GYRO_RANGE
{
    MPU6XXX_GYRO_RANGE_250DPS  = 0, // ±250°/s
    MPU6XXX_GYRO_RANGE_500DPS  = 1, // ±500°/s
    MPU6XXX_GYRO_RANGE_1000DPS = 2, // ±1000°/s
    MPU6XXX_GYRO_RANGE_2000DPS = 3  // ±2000°/s
} MPU_GYRO_RANGE;


/* Digital Low Pass Filter parameters */
typedef enum _MPU_DLPF
{
    MPU6XXX_DLPF_DISABLE = 0, //256HZ
    MPU6XXX_DLPF_188HZ = 1,
    MPU6XXX_DLPF_98HZ  = 2,
    MPU6XXX_DLPF_42HZ  = 3,
    MPU6XXX_DLPF_20HZ  = 4,
    MPU6XXX_DLPF_10HZ  = 5,
    MPU6XXX_DLPF_5HZ   = 6
} MPU_DLPF;

/* sleep mode parameters */
typedef enum _MPU_SLEEP
{
    MPU6XXX_SLEEP_DISABLE = 0,
    MPU6XXX_SLEEP_ENABLE  = 1
} MPU_SLEEP;

/* Supported configuration items */
enum mpu_cmd
{
    MPU_CMD_GYRO_RANGE,  /* Gyroscope full scale range */
    MPU_CMD_ACCEL_RANGE, /* Accelerometer full scale range */
    MPU_CMD_DLPF_CONFIG, /* Digital Low Pass Filter */
    MPU_CMD_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    MPU_CMD_SLEEP        /* Sleep mode */
};

typedef struct _XYZ_INT16T
{
  int16_t x;
  int16_t y;
  int16_t z;
} XYZ_INT16T;

typedef struct _MPU_CONFIG
{
  MPU_ACCEL_RANGE mpu_accel_range;
  MPU_GYRO_RANGE mpu_gyro_range;
  MPU_DLPF mpu_dlpf;
  MPU_SLEEP mpu_sleep;
  uint16_t mpu_sample_rate;
} MPU_CONFIG;

#define IMU_L_I2C_ADDR_SHIFTED  (MPU6XXX_ADDRESS_AD0_LOW << 1)
#define IMU_U_I2C_ADDR_SHIFTED  (MPU6XXX_ADDRESS_AD0_HIGH << 1)

HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_accel_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, XYZ_INT16T *xyz);
HAL_StatusTypeDef mpu_get_gyro_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, XYZ_INT16T *xyz);
HAL_StatusTypeDef mpu_get_accel_offset_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_gyro_offset_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
void mpu_sen_init();

#endif /* INC_COMMON_H_ */
