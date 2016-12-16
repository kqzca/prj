/**
  ******************************************************************************
  * @file           : usbd_dfu_if.c
  * @brief          :
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
/** @defgroup USBD_DFU 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_DFU_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Defines
  * @{
  */ 
#define FLASH_DESC_STR      "@Internal Flash   /0x08004000/1*16Kg"  
/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_IF_Private_Variables
  * @{
  */
/* USB handler declaration */
/* Handle for USB Full Speed IP */
  USBD_HandleTypeDef  *hUsbDevice_0;

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_DFU_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_DFU_Private_FunctionPrototypes
  * @{
  */
static uint16_t MEM_If_Init_FS(void);
static uint16_t MEM_If_Erase_FS (uint32_t Add);
static uint16_t MEM_If_Write_FS (uint8_t *src, uint8_t *dest, uint32_t Len);
static uint8_t *MEM_If_Read_FS  (uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_DeInit_FS(void);
static uint16_t MEM_If_GetStatus_FS (uint32_t Add, uint8_t Cmd, uint8_t *buffer);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET
void setGrnLed(GPIO_PinState state);
void setRedLed(GPIO_PinState state);
void delay(int counter);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN USBD_DFU_MediaTypeDef USBD_DFU_fops_FS __ALIGN_END =
{
   (uint8_t*)FLASH_DESC_STR,
    MEM_If_Init_FS,
    MEM_If_DeInit_FS,
    MEM_If_Erase_FS,
    MEM_If_Write_FS,
    MEM_If_Read_FS,
    MEM_If_GetStatus_FS,   
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  MEM_If_Init_FS
  *         Memory initialization routine.
  * @param  None
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Init_FS(void)
{ 
  /* USER CODE BEGIN 0 */ 
  return (USBD_OK);
  /* USER CODE END 0 */ 
}

/**
  * @brief  MEM_If_DeInit_FS
  *         De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_DeInit_FS(void)
{ 
  /* USER CODE BEGIN 1 */ 
  return (USBD_OK);
  /* USER CODE END 1 */ 
}

/**
  * @brief  MEM_If_Erase_FS
  *         Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Erase_FS(uint32_t Add)
{
  /* USER CODE BEGIN 2 */ 
	setGrnLed(LED_ON);
	
  /* Unlock the Flash to enable the flash control register access */ 
  HAL_FLASH_Unlock();
	
  uint32_t NbOfPages = (0x4000 + 1) >> 7;
  /* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = Add;
  EraseInitStruct.NbPages = NbOfPages;
	
	USBD_StatusTypeDef rtn = USBD_OK;
	uint32_t PageError = 0;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
		rtn = USBD_FAIL;
	}
	
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) */
  HAL_FLASH_Lock();
	
	setGrnLed(LED_OFF);
  return rtn;
  /* USER CODE END 2 */ 
}

/**
  * @brief  MEM_If_Write_FS
  *         Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* USER CODE BEGIN 3 */ 
	setRedLed(LED_ON);
  /* Unlock the Flash to enable the flash control register access */ 
  HAL_FLASH_Unlock();
	setGrnLed(LED_ON);
	
	USBD_StatusTypeDef rtn = USBD_OK;
	uint32_t offset = 0;
  while ((offset < Len) && (rtn == USBD_OK))
  {
		uint32_t ADDR_32 = (uint32_t)(dest + offset);
		uint32_t DATA_32 = *(uint32_t *)(src + offset);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_32, DATA_32) != HAL_OK)
		{
			rtn = USBD_FAIL;
		}
    offset = offset + 4;
  }
	
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) */
  HAL_FLASH_Lock();
	
	setRedLed(LED_OFF);
	setGrnLed(LED_OFF);
	
  return rtn;
  /* USER CODE END 3 */ 
}

/**
  * @brief  MEM_If_Read_FS
  *         Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *MEM_If_Read_FS (uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  /* USER CODE BEGIN 4 */ 
	setGrnLed(LED_ON);
	
	uint32_t offset = 0;
	while (offset < Len)
	{
		*(uint32_t *)(dest + offset) = *(uint32_t *)(src + offset);
		offset = offset + 4;
	}
	
	setGrnLed(LED_OFF);
	
  return dest;
  /* USER CODE END 4 */ 
}

/**
  * @brief  Flash_If_GetStatus_FS
  *         Get status routine.
  * @param  Add: Address to be read from.
  * @param  Cmd: Number of data to be read (in bytes).
  * @param  buffer: used for returning the time necessary for a program or an erase operation
  * @retval 0 if operation is successful
  */
uint16_t MEM_If_GetStatus_FS (uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
  /* USER CODE BEGIN 5 */ 
  switch (Cmd)
  {
  case DFU_MEDIA_PROGRAM:

    break;
    
  case DFU_MEDIA_ERASE:
  default:

    break;
  }                             
  return  (USBD_OK);
  /* USER CODE END 5 */  
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

void setGrnLed(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, state);
}

void setRedLed(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);
}

void delay(int counter)
{
	while (counter > 0)
	{
		int a = 1000;
		while (a > 0)
		{
			a--;
		}
		counter--;
	}
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

