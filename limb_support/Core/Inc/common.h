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

uint32_t get_counter();
void increase_counter();
uint8_t is_counter_unchanged();

GPIO_PinState read_key0();
GPIO_PinState read_key1();
GPIO_PinState read_ext_sw();
void write_LED0(GPIO_PinState state);
void write_LED1(GPIO_PinState state);
void write_LEDExt(GPIO_PinState state);

void srat_ADC(ADC_HandleTypeDef* hadc, uint32_t channel);
uint32_t read_ADC(ADC_HandleTypeDef* hadc);

/* Accelerometer full scale range */
enum mpu_accel_range
{
    MPU6XXX_ACCEL_RANGE_2G  = 0, // ±2G
    MPU6XXX_ACCEL_RANGE_4G  = 1, // ±4G
    MPU6XXX_ACCEL_RANGE_8G  = 2, // ±8G
    MPU6XXX_ACCEL_RANGE_16G = 3  // ±16G
};

/* Gyroscope full scale range */
enum mpu_gyro_range
{
    MPU6XXX_GYRO_RANGE_250DPS  = 0, // ±250°/s
    MPU6XXX_GYRO_RANGE_500DPS  = 1, // ±500°/s
    MPU6XXX_GYRO_RANGE_1000DPS = 2, // ±1000°/s
    MPU6XXX_GYRO_RANGE_2000DPS = 3  // ±2000°/s
};

/* Digital Low Pass Filter parameters */
enum mpu_dlpf
{
    MPU6XXX_DLPF_DISABLE = 0, //256HZ
    MPU6XXX_DLPF_188HZ = 1,
    MPU6XXX_DLPF_98HZ  = 2,
    MPU6XXX_DLPF_42HZ  = 3,
    MPU6XXX_DLPF_20HZ  = 4,
    MPU6XXX_DLPF_10HZ  = 5,
    MPU6XXX_DLPF_5HZ   = 6
};

/* sleep mode parameters */
enum mpu_sleep
{
    MPU6XXX_SLEEP_DISABLE = 0,
    MPU6XXX_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum mpu_cmd
{
    MPU6XXX_GYRO_RANGE,  /* Gyroscope full scale range */
    MPU6XXX_ACCEL_RANGE, /* Accelerometer full scale range */
    MPU6XXX_DLPF_CONFIG, /* Digital Low Pass Filter */
    MPU6XXX_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    MPU6XXX_SLEEP        /* Sleep mode */
};

#define IMU_L_I2C_ADDR_SHIFTED  (MPU6XXX_ADDRESS_AD0_LOW << 1)
#define IMU_U_I2C_ADDR_SHIFTED  (MPU6XXX_ADDRESS_AD0_HIGH << 1)

HAL_StatusTypeDef i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_read_regs(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t reg, uint8_t len, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_accel_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_gyro_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_accel_offset_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_get_gyro_offset_buf(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t *buffer);
HAL_StatusTypeDef mpu_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);

#endif /* INC_COMMON_H_ */
