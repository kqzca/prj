/**
  ******************************************************************************
  * File Name          : i2c_lis3dsh.c
  * Description        : LIS3DSH related operations using I2C
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "i2c_lis3dsh.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

/* Private */
extern void LIS3DSH_INT_Write(uint8_t* data, uint8_t addr, uint8_t count);
extern void LIS3DSH_INT_Read(uint8_t* data, uint8_t addr, uint8_t count);
extern void LIS3DSH_INT_InitLIS3DSH(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter);
extern void LIS3DSH_INT_InitLIS302DL(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter);
extern void LIS3DSH_INT_ReadAxes(LIS3DSH_t* Axes_Data);
extern void INT_ReadAxes(LIS3DSH_t* Axes_Data);
extern void LIS3DSH_INT_Delay(void);

LIS3DSH_Device_t LIS3DSH_INT_Device = LIS3DSH_Device_Error;
float LIS3DSH_INT_Sensitivity;

/* Public */
uint8_t LIS3DSH_Detect(void)
{
	uint8_t id;
	/* Delay on power up */
	LIS3DSH_INT_Delay();
	/* Get ID */
	LIS3DSH_INT_Read(&id, LIS3DSH_REG_WHO_I_AM, 1);
	/* Check device */
	if (id == LIS3DSH_ID) {
		/* Set device */
		LIS3DSH_INT_Device = LIS3DSH_Device_LIS3DSH;
		/* Return device */;
		return LIS3DSH_Device_LIS3DSH;
	}
	
	/* Return Error */
	return LIS3DSH_Device_Error;
}

uint8_t LIS3DSH_Init(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter)
{
	/* Some delay */
	LIS3DSH_INT_Delay();
	/* Detect proper device and init it */
	if (LIS3DSH_Detect() == LIS3DSH_Device_LIS3DSH)
  {
		/* Init sequence for LIS3DSH */
		LIS3DSH_INT_InitLIS3DSH(Sensitivity, Filter);
		/* Return device */
		return LIS3DSH_Device_LIS3DSH;
	}
	
	/* Error detection */
	LIS3DSH_INT_Device = LIS3DSH_Device_Error;
	/* Return Error */
	return LIS3DSH_Device_Error;
}

uint8_t LIS3DSH_ReadAxes(LIS3DSH_t* Axes_Data)
{
	if (LIS3DSH_INT_Device == LIS3DSH_Device_LIS3DSH)
  {
		/* Init sequence for LIS3DSH */
		LIS3DSH_INT_ReadAxes(Axes_Data);
		/* Return device */
		return LIS3DSH_Device_LIS3DSH;
	}
	/* Return Error */
	return LIS3DSH_Device_Error;
}

void LIS3DSH_INT_Write(uint8_t* data, uint8_t addr, uint8_t count)
{
  uint8_t tx_buf[16];
  tx_buf[0] = addr;
  count = count <= 15 ? count : 15;
  memcpy(tx_buf+1, data, count);
  HAL_I2C_Master_Transmit(&hi2c1, LIS3DSH_ADDR, tx_buf, count + 1, 5);
}

void LIS3DSH_INT_Read(uint8_t* data, uint8_t addr, uint8_t count)
{
	/* Add read bit */
	addr |= 0x80;
	
	if (count > 1)
  {
		/* Add autoincrement bit */
		addr |= 0x40;
	}
	
  HAL_I2C_Master_Transmit(&hi2c1, LIS3DSH_ADDR, &addr, 1, 5);
  HAL_I2C_Master_Receive(&hi2c1, LIS3DSH_ADDR, data, count, 5);
}

void LIS3DSH_INT_InitLIS3DSH(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter) {
	uint8_t tmpreg;
	uint16_t temp;

	/* Set data */
	temp = (uint16_t) (LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE);
	temp |= (uint16_t) (LIS3DSH_SERIALINTERFACE_4WIRE | LIS3DSH_SELFTEST_NORMAL);
	
	/* Set sensitivity */
	if (Sensitivity == LIS3DSH_Sensitivity_2G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_2);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_4G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_4);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_6G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_6);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_8G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_8);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_16G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_16);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
	} else {
		return;
	}
	
	/* Set filter */
	if (Filter == LIS3DSH_Filter_800Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_800 << 8);
	} else if (Filter == LIS3DSH_Filter_400Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_400 << 8);
	} else if (Filter == LIS3DSH_Filter_200Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_200 << 8);
	} else if (Filter == LIS3DSH_Filter_50Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_50 << 8);
	} else {
		return;
	}
	
	/* Configure MEMS: power mode(ODR) and axes enable */
	tmpreg = (uint8_t) (temp);

	/* Write value to MEMS CTRL_REG4 register */
	LIS3DSH_INT_Write(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);

	/* Configure MEMS: full scale and self test */
	tmpreg = (uint8_t) (temp >> 8);

	/* Write value to MEMS CTRL_REG5 register */
	LIS3DSH_INT_Write(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
}

void LIS3DSH_INT_ReadAxes(LIS3DSH_t *Axes_Data) {
	int8_t buffer[6];

	LIS3DSH_INT_Read((uint8_t*)&buffer[0], LIS3DSH_OUT_X_L_ADDR, 1);
	LIS3DSH_INT_Read((uint8_t*)&buffer[1], LIS3DSH_OUT_X_H_ADDR, 1);
	LIS3DSH_INT_Read((uint8_t*)&buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1);
	LIS3DSH_INT_Read((uint8_t*)&buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1);
	LIS3DSH_INT_Read((uint8_t*)&buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1);
	LIS3DSH_INT_Read((uint8_t*)&buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1);
	
	/* Set axes */
	Axes_Data->X = (int16_t)((buffer[1] << 8) + buffer[0]) * LIS3DSH_INT_Sensitivity;
	Axes_Data->Y = (int16_t)((buffer[3] << 8) + buffer[2]) * LIS3DSH_INT_Sensitivity;
	Axes_Data->Z = (int16_t)((buffer[5] << 8) + buffer[4]) * LIS3DSH_INT_Sensitivity;
}

void LIS3DSH_INT_Delay(void) {
	uint32_t delay = 72000;
	while (delay--);
}

