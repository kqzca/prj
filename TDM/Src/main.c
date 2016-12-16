/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#include "stm32f3xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "spi_sd.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t txOverUsbUart = 0;
uint8_t usbComOutBuf[USB_COM_BUFSIZE];
uint8_t scanIntervalMs = 10;
uint64_t sdWriteAddr = SD_START_ADDR;
uint8_t wrbuf[SD_BLOCK_SIZE];
uint8_t rdBuf[SD_BLOCK_SIZE];
extern uint8_t is_addr_block_idx;
  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t sdDetected = SD_NOT_PRESENT;
  uint32_t oldTick = 0;
  uint16_t bufDataPos = 0;  // 0 <= pos < SD_BLOCK_SIZE
  SNR_DATA_t dataBlock;
  memset(wrbuf, 0, SD_BLOCK_SIZE);
  LED_R_OFF();
  while (1)
  {
    checkUartBuf();
    uint8_t sdDetect = SD_Detect();
    if (sdDetected != sdDetect)
    {
      if (sdDetect == SD_PRESENT)
      {
        LED_Y_OFF();
        SD_CS_HIGH();
        SD_Init();
        SD_CardInfo cardinfo;
        SD_Error err = SD_GetCardInfo(&cardinfo);
        LIS3DSH_Detect();
        LIS3DSH_Init(LIS3DSH_Sensitivity_2G, LIS3DSH_Filter_800Hz);
        l3g_init();
        l3g_enableDefault();
      }
    }
    sdDetected = sdDetect;
    if (sdDetected != SD_PRESENT)
    {
      LED_Y_ON();
      continue;
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    uint32_t newTick = HAL_GetTick();
    if (newTick - oldTick < scanIntervalMs)
    {
      continue;
    }
    
    oldTick = newTick;
    
    LIS3DSH_ReadAxes(&(dataBlock.dataLIS3DSH));
    l3g_read(&(dataBlock.dataL3GD20H));
    
    if (txOverUsbUart == 1)
    {
      if (!USB_DET())
      {
        txOverUsbUart = 0;
      }
      else
      {
        memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "%6d,%6d,%6d,%6d,%6d,%6d\n",
                dataBlock.dataLIS3DSH.X, dataBlock.dataLIS3DSH.Y, dataBlock.dataLIS3DSH.Z,
                dataBlock.dataL3GD20H.x, dataBlock.dataL3GD20H.y, dataBlock.dataL3GD20H.z);
        CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
      }
    }
    
    memcpy(wrbuf + bufDataPos, &dataBlock, sizeof(SNR_DATA_t));
    bufDataPos += sizeof(SNR_DATA_t);
    if ((bufDataPos + sizeof(SNR_DATA_t)) >= SD_BLOCK_SIZE)
    {
      SD_WriteBlock(wrbuf, sdWriteAddr, SD_BLOCK_SIZE);
      SD_ReadBlock(rdBuf, sdWriteAddr, SD_BLOCK_SIZE);
      if (memcmp(wrbuf, rdBuf, SD_BLOCK_SIZE) != 0)
      {
        Error_Handler();
      }
      if ((sdWriteAddr / SD_BLOCK_SIZE) % 2 == 0)
      {
        LED_G_ON();
      }
      else
      {
        LED_G_OFF();
      }
      sdWriteAddr += SD_BLOCK_SIZE;
      bufDataPos = 0;
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

void checkUartBuf(void)
{
	uint8_t rxMsg[RXCHRBUFSIZE];
	if (getMsg(rxMsg, RXCHRBUFSIZE) > 0)
	{
		int newSetValue = 0;
    int printIndex = 0;
    uint64_t readCmdAddr = 0;
		switch (rxMsg[0])
		{
			case 'p':
			case 'P':
				txOverUsbUart = 0;
				break;
			case 't':
			case 'T':
				txOverUsbUart = 1;
				break;
			case 'i':
			case 'I':
				switch (rxMsg[1])
				{
					case 0:
						break;
					default:
						newSetValue = atoi((const char *)&(rxMsg[1]));
						if ((newSetValue > 0) && (newSetValue < 100) && (newSetValue != scanIntervalMs))
						{
							scanIntervalMs = newSetValue;
						}
						break;
				}
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "scanIntervalMs(ms) = %d\n", scanIntervalMs);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
			case 's':
			case 'S':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "is_addr_block_idx=%d Current block 0x%012llX\n",
                is_addr_block_idx, sdWriteAddr);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
			case 'h':
			case 'H':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "T/t    : start data flow on serial port\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "P/p    : stop data flow on serial port\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "I/i    : show scan interval\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "I/ix   : set the scan interval to x ms\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "         example: i15 will set the scan interval to 15 ms\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "S/s    : show SD card start address and current block address\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "R/raddr: read a block at addr\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "         example: r0x00001000 (Note: no space)\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_Delay(1);
				break;
			case 'r':
			case 'R':
				switch (rxMsg[1])
				{
					case 0:
            readCmdAddr = 0ll;
						break;
					default:
						readCmdAddr = strtoull((const char *)&(rxMsg[1]), 0, 16);
						break;
				}
        SD_ReadBlock(rdBuf, readCmdAddr, SD_BLOCK_SIZE);
        memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
        sprintf((char *)usbComOutBuf, "Read addr = 0x%012llX\n", readCmdAddr);
        printIndex = 0;
        CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        while ((printIndex + 12) < SD_BLOCK_SIZE)
        {
          memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
          sprintf((char *)usbComOutBuf, "%6d,%6d,%6d,%6d,%6d,%6d\n",
            *(short *)(rdBuf + printIndex),     *(short *)(rdBuf + printIndex + 2), *(short *)(rdBuf + printIndex + 4),
            *(short *)(rdBuf + printIndex + 6), *(short *)(rdBuf + printIndex + 8), *(short *)(rdBuf + printIndex + 10));
          printIndex += 12;
          CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        }
				break;
			default:
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "Command \'%c\' is not valid.\n", rxMsg[0]);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  LED_R_ON();
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
