 /**
  ******************************************************************************
  * @file    						:	spi_sd.h
  * @description        : SD read/write functions using SPI
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TDM_MAIN_H
#define TDM_MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "i2c_lis3dsh.h"
#include "i2c_l3gd20h.h"

typedef struct WR_DATA_BLOCK_16_s
{
  LIS3DSH_t dataLIS3DSH;
  L3GD20H_t dataL3GD20H;
} SNR_DATA_t;

#define LED_R_ON()	(HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET))
#define LED_R_OFF() (HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET))
#define LED_Y_ON()	(HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET))
#define LED_Y_OFF() (HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET))
#define LED_G_ON()	(HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET))
#define LED_G_OFF() (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))
#define USB_DET()   (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)

#define USB_COM_BUFSIZE 128
void checkUartBuf(void);

#ifdef __cplusplus
}
#endif

#endif /* TDM_MAIN_H */
