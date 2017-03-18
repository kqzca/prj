/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const uint16_t BUFSIZE = 20;
static const uint16_t TIMEOUT = 500;
static const uint8_t scanIntervalMs = 10;
static uint8_t rxByte = 0, rxBufIdx = 0;
static uint8_t rxBuf[BUFSIZE];
static uint8_t txBuf[BUFSIZE];
static uint8_t data[BUFSIZE] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9};
static uint16_t counterBT = 0, timerBT = 0;
uint8_t txBleUart = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void checkUsbComBuf(void);
#define USB_COM_BUFSIZE 128
uint8_t usbComOutBuf[USB_COM_BUFSIZE];

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

typedef enum
{
  StateOff = 0,
  StateOn
} StateOnOff;

static void setLed(uint8_t ledIdx, StateOnOff state)
{
  switch (ledIdx)
  {
    case 1:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)state);
    break;
    case 2:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  (GPIO_PinState)state);
    break;
    case 3:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  (GPIO_PinState)state);
    break;
  }
}

static void setBlePwr(StateOnOff state)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, (state == StateOn) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void setBleWake(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, state);
}

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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  setLed(1, StateOn);
  setBlePwr(StateOn);
  memset(txBuf, 0, BUFSIZE);
  memset(rxBuf, 0, BUFSIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LIS3DSH_Detect();
  LIS3DSH_Init(LIS3DSH_Sensitivity_2G, LIS3DSH_Filter_800Hz);
  l3g_init();
  l3g_enableDefault();
  uint32_t oldTick = 0;
  uint8_t led2State = StateOn;
  SNR_DATA_t dataBlock;
  
  HAL_UART_Receive_DMA(&huart6, &rxByte, 1);

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    uint32_t newTick = HAL_GetTick();
    if (newTick - oldTick < scanIntervalMs)
    {
      continue;
    }
    
    checkUsbComBuf();
    
    oldTick = newTick;
    timerBT++;
    
    LIS3DSH_ReadAxes(&(dataBlock.dataLIS3DSH));
    l3g_read(&(dataBlock.dataL3GD20H));
    
    if ((dataBlock.dataLIS3DSH.X == 0 && dataBlock.dataLIS3DSH.Y == 0 && dataBlock.dataLIS3DSH.Z == 0) ||
        (dataBlock.dataL3GD20H.x == 0 && dataBlock.dataL3GD20H.y == 0 && dataBlock.dataL3GD20H.z == 0))
      Error_Handler();
    
    if (TIMEOUT <= timerBT * scanIntervalMs)
    {
      timerBT = 0;
      setLed(2, led2State);
      led2State = !led2State;
      memcpy(txBuf + 2, data + 2, 18);
      *((uint16_t *)txBuf) = counterBT++;
      if (txBleUart)
        HAL_UART_Transmit(&huart6, (uint8_t*)txBuf, BUFSIZE, TIMEOUT);
      
      int i=0;
      memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
      while (i<rxBufIdx)
      {
        sprintf((char *)usbComOutBuf + i*5, "0x%02X ", rxBuf[i]);
        i++;
      }
      rxBufIdx = 0;
      if (i > 0)
      {
        sprintf((char *)usbComOutBuf + i*5, "---\n");
        CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
      }
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

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void checkUsbComBuf(void)
{
	uint8_t rxMsg[RXCHRBUFSIZE];
  uint8_t addrCmd[8] = {0x00, 0x04, 0x09, 0x01, 0x00, 0x10, 0x00, 0x00};
  uint8_t dfuResetCmd[5]  = {0x00, 0x01, 0x09, 0x00, 0x01};
  uint8_t sysResetCmd[5]  = {0x00, 0x01, 0x00, 0x00, 0x01};
	if (getMsg(rxMsg, RXCHRBUFSIZE) > 0)
	{
		switch (rxMsg[0])
		{
			case 'p':
			case 'P':
				txBleUart = 0;
				break;
			case 't':
			case 'T':
				txBleUart = 1;
				break;
			case 'r':
			case 'R':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "BLE Reset.\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        setLed(1, StateOff);
        setBlePwr(StateOff);
        HAL_Delay(500);
        setLed(1, StateOn);
        setBlePwr(StateOn);
        HAL_Delay(500);
        HAL_UART_Transmit(&huart6, addrCmd, 8, TIMEOUT);
				break;
			case 'h':
			case 'H':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "BLE P0_0 High.\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        setBleWake(GPIO_PIN_SET);
				break;
			case 'l':
			case 'L':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "BLE P0_0 Low.\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        setBleWake(GPIO_PIN_RESET);
				break;
			case 'a':
			case 'A':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "BLE ADDR Command.\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_UART_Transmit(&huart6, addrCmd, 8, TIMEOUT);
				break;
			case 'd':
			case 'D':
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "BLE DFU Command.\n");
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
        HAL_UART_Transmit(&huart6, dfuResetCmd, 5, TIMEOUT);
				break;
			default:
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "Command \'%c\' is not valid.\n", rxMsg[0]);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  rxBuf[rxBufIdx++] = rxByte;
  HAL_UART_Receive_DMA(&huart6, &rxByte, 1);
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
