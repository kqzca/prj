 /**
  ******************************************************************************
  * @file    						:	spi_sd.h
  * @description        : SD read/write functions using SPI
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TDMv3_MAIN_H
#define TDMv3_MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "i2c_lis3dsh.h"
#include "i2c_l3gd20h.h"

typedef struct WR_DATA_BLOCK_16_s
{
  LIS3DSH_t dataLIS3DSH;
  L3GD20H_t dataL3GD20H;
} SNR_DATA_t;

#ifdef __cplusplus
}
#endif

#endif /* TDMv3_MAIN_H */
