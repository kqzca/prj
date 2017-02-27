/**
  ******************************************************************************
  * File Name          : i2c_l3gd20h.c
  * Description        : L3GD20H related operations using I2C
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "i2c_l3gd20h.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
uint8_t address = L3GD20H_ADDRESS;
uint32_t io_timeout = 5;
uint8_t did_timeout = 0;
float L3G_Sensitivity;

uint8_t l3g_timeoutOccurred(void)
{
  uint8_t tmp = did_timeout;
  did_timeout = 0;
  return tmp;
}

void l3g_setTimeout(uint32_t timeout)
{
  io_timeout = timeout;
}

uint32_t l3g_getTimeout()
{
  return io_timeout;
}

uint8_t l3g_init(void)
{
  int id = l3g_testReg(WHO_AM_I);
  if (id != L3GD20H_WHO_ID)
  {
    return 0;
  }
  return 1;
}

void l3g_enableDefault(void)
{
  // 0x00 = 0b00000000
  // Low_ODR = 0 (low speed ODR disabled)
  l3g_writeReg(LOW_ODR, 0x00);
  
  // 0x00 = 0b00000000
  // FS = 00 (+/- 250 dps full scale)
  l3g_writeReg(CTRL4, 0x00);
  L3G_Sensitivity = GYRO_SENSITIVITY_250DPS;
  
  // 0x6F = 0b01101111
  // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  l3g_writeReg(CTRL1, 0x6F);
}

void l3g_writeReg(uint8_t reg, uint8_t value)
{
  uint8_t tx_buf[2];
  tx_buf[0] = reg;
  tx_buf[1] = value;
  HAL_I2C_Master_Transmit(&hi2c1, address, tx_buf, 2, io_timeout);
}

uint8_t l3g_readReg(uint8_t reg)
{
  uint8_t value;
  HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, io_timeout);
  HAL_I2C_Master_Receive(&hi2c1, address, &value, 1, io_timeout);
  return value;
}

void l3g_read(L3GD20H_t *g)
{
  if (g != 0)
  {
    uint8_t addr = OUT_X_L | (1 << 7);
    uint8_t data[6];
    HAL_I2C_Master_Transmit(&hi2c1, address, &addr, 1, io_timeout);
    HAL_I2C_Master_Receive(&hi2c1, address, data, 6, io_timeout);
    
    uint8_t xlg = data[0];
    uint8_t xhg = data[1];
    uint8_t ylg = data[2];
    uint8_t yhg = data[3];
    uint8_t zlg = data[4];
    uint8_t zhg = data[5];

    // combine high and low bytes
    g->x = (short)(L3G_Sensitivity * (int16_t)(xhg << 8 | xlg));
    g->y = (short)(L3G_Sensitivity * (int16_t)(yhg << 8 | ylg));
    g->z = (short)(L3G_Sensitivity * (int16_t)(zhg << 8 | zlg));
  }
}
int l3g_testReg(regAddr_t reg)
{
  return l3g_readReg(reg);
}

