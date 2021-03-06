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
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "rtc.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//#define USE_SWSIG
const int DAC_OUT_TO_ADC_IN_DELAY = 10;
const uint32_t EEPROM_ADDR_SNRDRVVAL = 0;
const int SNR_CAL_DAC_OFFSET = 0x0fff / 30;
const int SNR_PASS_THRESHOLD = 0x0fff / 2;

typedef enum
{
  STATE_WAKEUP = 0,
  STATE_SLEEP
} Wakeup_Sleep_State;
Wakeup_Sleep_State wakeupSleepState = STATE_WAKEUP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

#define BUTTON_PRESSED GPIO_PIN_RESET
#define BUTTON_RELEASED GPIO_PIN_SET
GPIO_PinState checkPushButton(void);
typedef enum
{
  BTNFUNC_NONE = 0,
  BTNFUNC_STATECHN,
	BTNFUNC_CAL,
} Button_Function;
Button_Function checkButtonFunction(void);
#define USB_PWR_DETECTED GPIO_PIN_SET
GPIO_PinState checkUsbPower(void);
void goSleep(void);

#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET
void setSysWakeUpLed(GPIO_PinState state);	// Green
void setSysStsLed(GPIO_PinState state); // Red
void setTestFailLed(GPIO_PinState state); // Red
void setTestPassLed(GPIO_PinState state); // Green

#define DEV_PWR_ON GPIO_PIN_RESET
#define DEV_PWR_OFF GPIO_PIN_SET
void setBlueToothDevPwr(GPIO_PinState state);
void setSensorDevPwr(GPIO_PinState state);

#define BLESIG_HI GPIO_PIN_SET
#define BLESIG_LO GPIO_PIN_RESET
void setBleSig(GPIO_PinState state);

int readAdcChannel(uint32_t channel);
int dacOutAdcIn(int dacValue, uint32_t adcChannel);
void getSnrDrvDacVal(void);
HAL_StatusTypeDef writeEEPRom32(uint32_t address, uint32_t data);
uint32_t readEEPROM32(uint32_t address);

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
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
				
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//int loopCnt = 0;
  while (1)
  {
		Button_Function buttonFunction = checkButtonFunction();
		if (buttonFunction == BTNFUNC_STATECHN)
		{
			wakeupSleepState = (wakeupSleepState == STATE_WAKEUP) ? STATE_SLEEP : STATE_WAKEUP;
		}
		
		if (buttonFunction == BTNFUNC_CAL)
		{
			setSensorDevPwr(DEV_PWR_ON);
			getSnrDrvDacVal();
			setSensorDevPwr(DEV_PWR_OFF);
			wakeupSleepState = STATE_WAKEUP;
		}
				
		if (wakeupSleepState == STATE_WAKEUP)
		{
			uint16_t snrAdcVal = 0;
			
			setBlueToothDevPwr(DEV_PWR_ON);
			setSensorDevPwr(DEV_PWR_ON);
			setSysWakeUpLed(LED_ON);
			setSysStsLed(LED_OFF);
			
			int snrDrvDacVal = readEEPROM32(EEPROM_ADDR_SNRDRVVAL);
			if ((snrDrvDacVal < 1) || (snrDrvDacVal > 0xFFF))
			{
				getSnrDrvDacVal();
			}
			snrAdcVal = (uint16_t)dacOutAdcIn(snrDrvDacVal, ADC_CHANNEL_8);
			if (snrAdcVal > SNR_PASS_THRESHOLD)
			{
				setTestPassLed(LED_ON);
				setTestFailLed(LED_OFF);
			}
			else
			{
				setTestFailLed(LED_ON);
				setTestPassLed(LED_OFF);
			}
			
			if (hi2c1.State == HAL_I2C_STATE_READY)
			{
				setBleSig(BLESIG_HI);
				if (HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t *)&snrAdcVal, 2, 3000) == HAL_OK)
				{
					setSysStsLed(LED_ON);
					HAL_Delay(100);
					setSysStsLed(LED_OFF);
					HAL_Delay(100);
					setSysStsLed(LED_ON);
					HAL_Delay(100);
					setSysStsLed(LED_OFF);
				}
				else
				{
					setSysStsLed(LED_ON);
					HAL_Delay(300);
					setSysStsLed(LED_OFF);
				}
			}
			else
			{
				setSysWakeUpLed(LED_OFF);
				HAL_Delay(100);
				setSysWakeUpLed(LED_ON);
				HAL_Delay(100);
				setSysWakeUpLed(LED_OFF);
				HAL_Delay(100);
				setSysWakeUpLed(LED_ON);
				HAL_Delay(100);
			}
			
			setTestFailLed(LED_OFF);
			setTestPassLed(LED_OFF);
			HAL_Delay(1000);
			setBleSig(BLESIG_LO);
			HAL_Delay(1000);
		}
		else
		{
			goSleep();
		}
		
		/*
		loopCnt++;
		if (loopCnt >= 2)
		{
			loopCnt = 0;
		}
		*/
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

GPIO_PinState checkPushButton(void)
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
}

Button_Function checkButtonFunction(void)
{
	Button_Function buttonFunction = BTNFUNC_NONE;
	GPIO_PinState buttonState = checkPushButton();
	if (buttonState == BUTTON_PRESSED)
	{
		setSysStsLed(LED_ON);
		HAL_Delay(1000);
		GPIO_PinState buttonState = checkPushButton();
		if (buttonState == BUTTON_PRESSED)
		{
			buttonFunction = BTNFUNC_STATECHN;
			setSysStsLed(LED_OFF);
			HAL_Delay(1000);
			GPIO_PinState buttonState = checkPushButton();
			if (buttonState == BUTTON_PRESSED)
			{
				setSysStsLed(LED_ON);
				HAL_Delay(1000);
				buttonFunction = BTNFUNC_CAL;
			}
		}
	}
	return buttonFunction;
}
		
void goSleep(void)
{
	setSensorDevPwr(DEV_PWR_OFF);
	setSysWakeUpLed(LED_OFF);
	setSysStsLed(LED_OFF);
	setTestPassLed(LED_OFF);
	setTestFailLed(LED_OFF);
	
	int adcVolBat = 0;
	do
	{
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		HAL_ResumeTick();
		adcVolBat = readAdcChannel(ADC_CHANNEL_7);
		if (adcVolBat < (0x0400))
		{
			setSysStsLed(LED_ON);
			HAL_Delay(10);
			setSysStsLed(LED_OFF);
			HAL_Delay(200);
			setSysStsLed(LED_ON);
			HAL_Delay(10);
			setSysStsLed(LED_OFF);
		}
	} while (adcVolBat < (0x0400));
}

GPIO_PinState checkUsbPower(void)
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}

void setSysWakeUpLed(GPIO_PinState state)	// Green
{
#ifdef USE_SWSIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, state);
#else
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, state);
#endif
}

void setSysStsLed(GPIO_PinState state) // Red
{
#ifdef USE_SWSIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);
#else
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, state);
#endif
}

void setTestFailLed(GPIO_PinState state) // Red
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);
}

void setTestPassLed(GPIO_PinState state)	// Green
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, state);
}

void setBlueToothDevPwr(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state);
}

void setSensorDevPwr(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state);
}

void setBleSig(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, state);
}

int readAdcChannel(uint32_t channel)
{
	int rtn = -1;
	
  HAL_ADC_Init(&hadc);
  ADC_ChannelConfTypeDef sConfig;
	
	static uint32_t channels[3] = {ADC_CHANNEL_7, ADC_CHANNEL_9, ADC_CHANNEL_9};
	for (int idx = 0; idx < 3; idx++)
	{
		sConfig.Channel = channels[idx];
		if (channel == sConfig.Channel)
		{
			sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		}
		else
		{
			sConfig.Rank = ADC_RANK_NONE;
		}
		HAL_ADC_ConfigChannel(&hadc, &sConfig);
	}
	
	int adcRead[3] = {-1, -1, -1};
	for (int i=0; i<3; i++)
	{
		if (HAL_ADC_Start(&hadc) == HAL_OK)
		{
			if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK)
			{
				adcRead[i] = (int)HAL_ADC_GetValue(&hadc);
				HAL_ADC_Stop(&hadc);
			}
			else
			{
				HAL_ADC_Stop(&hadc);
				break;
			}
		}
		else
		{
			break;
		}
	}
	
	if ((adcRead[0] >= 0) && (adcRead[1] >= 0) && (adcRead[2] >= 0))
	{
		rtn = (adcRead[0] + adcRead[1] + adcRead[2])/3;
	}
	return rtn;
}

int dacOutAdcIn(int dacValue, uint32_t adcChannel)
{
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);
	HAL_Delay(DAC_OUT_TO_ADC_IN_DELAY); // ms
	int adcValue = readAdcChannel(adcChannel);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	return adcValue;
}

void getSnrDrvDacVal()
{
	int maxAdc = 0;
	int snrDrvDacVal = 0;
	dacOutAdcIn(0, ADC_CHANNEL_9);
	setSysWakeUpLed(LED_ON);
	setSysStsLed(LED_ON);
	HAL_Delay(3000);
	setSysWakeUpLed(LED_OFF);
	setSysStsLed(LED_OFF);
#ifdef USE_SWSIG
	for (int dacStep = 0x0000; dacStep <= 0x0fff; dacStep += 0x10)
#else
	for (int dacStep = 0x0000; dacStep <= 0x0fff; dacStep++)
#endif
	{
		int adcValue = dacOutAdcIn(dacStep, ADC_CHANNEL_9);
		if (adcValue > maxAdc)
		{
			maxAdc = adcValue;
			snrDrvDacVal = dacStep;
		}
		setSysWakeUpLed((dacStep & 0x0100) ? LED_ON : LED_OFF);
		setSysStsLed((dacStep & 0x0200) ? LED_ON : LED_OFF);
		setTestFailLed((dacStep & 0x0400) ? LED_ON : LED_OFF);
		setTestPassLed((dacStep & 0x0800) ? LED_ON : LED_OFF);
	}
	if (snrDrvDacVal < (0xfff - SNR_CAL_DAC_OFFSET))
	{
		snrDrvDacVal += SNR_CAL_DAC_OFFSET;
	}
	writeEEPRom32(EEPROM_ADDR_SNRDRVVAL, (uint32_t)snrDrvDacVal);
	setSysWakeUpLed(LED_OFF);
	setSysStsLed(LED_OFF);
	setTestFailLed(LED_OFF);
	setTestPassLed(LED_OFF);
}

HAL_StatusTypeDef writeEEPRom32(uint32_t address, uint32_t data)
{
	HAL_StatusTypeDef status = HAL_FLASHEx_DATAEEPROM_Unlock();
	if (status == HAL_OK)
	{
		status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address + DATA_EEPROM_BASE, data);
	}
	HAL_FLASHEx_DATAEEPROM_Lock();
  return status;
}
 
uint32_t readEEPROM32(uint32_t address)
{
	return *(__IO uint32_t*)(address + DATA_EEPROM_BASE);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	
}

/* USER CODE END 4 */

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
