 /**
  ******************************************************************************
  * @file    						:	i2c_l3gd20h.h
  * @description        : L3GD20H related operations using I2C
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_L3GD20H_H
#define I2C_L3GD20H_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define L3GD20H_ADDRESS   0xD4
#define L3GD20H_WHO_ID    0xD7

#define TEST_REG_ERROR -1

// register addresses
typedef enum
{
  WHO_AM_I       = 0x0F,

  CTRL1          = 0x20, // D20H
  CTRL2          = 0x21, // D20H
  CTRL3          = 0x22, // D20H
  CTRL4          = 0x23, // D20H
  CTRL5          = 0x24, // D20H
  REFERENCE      = 0x25,
  OUT_TEMP       = 0x26,
  STATUS         = 0x27, // D20H

  OUT_X_L        = 0x28,
  OUT_X_H        = 0x29,
  OUT_Y_L        = 0x2A,
  OUT_Y_H        = 0x2B,
  OUT_Z_L        = 0x2C,
  OUT_Z_H        = 0x2D,

  FIFO_CTRL      = 0x2E, // D20H
  FIFO_SRC       = 0x2F, // D20H

  IG_CFG         = 0x30, // D20H
  IG_SRC         = 0x31, // D20H
  IG_THS_XH      = 0x32, // D20H
  IG_THS_XL      = 0x33, // D20H
  IG_THS_YH      = 0x34, // D20H
  IG_THS_YL      = 0x35, // D20H
  IG_THS_ZH      = 0x36, // D20H
  IG_THS_ZL      = 0x37, // D20H
  IG_DURATION    = 0x38, // D20H

  LOW_ODR        = 0x39  // D20H
} regAddr_t;

#define GYRO_SENSITIVITY_250DPS  (0.875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (1.75F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (7.0F)      // Roughly 18/256

typedef struct {

	int16_t x;
	int16_t y;
	int16_t z;
} L3GD20H_t;

uint8_t l3g_init(void);
void l3g_enableDefault(void);
void l3g_writeReg(uint8_t reg, uint8_t value);
uint8_t l3g_readReg(uint8_t reg);
void l3g_read(L3GD20H_t *g);
void l3g_setTimeout(unsigned int timeout);
unsigned int l3g_getTimeout(void);
uint8_t l3g_timeoutOccurred(void);
int l3g_testReg(regAddr_t reg);

#ifdef __cplusplus
}
#endif

#endif /* I2C_L3GD20H_H */
