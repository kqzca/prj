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
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//#define USE_SWSIG
const int DAC_OUT_TO_ADC_IN_DELAY = 10;
const uint32_t EEPROM_ADDR_SNRDRVVAL = 0;
const uint32_t EEPROM_ADDR_OFFSET = 32;
const uint32_t EEPROM_ADDR_NORMVOUT = 36;
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

#define VDDAMV 3117
#define MilliVoltToAdcVal(x) ((float)(x)*4096.0/(float)VDDAMV)
#define ADCValToVolt(x) ((float)(x)*(float)VDDAMV/4095.0/1000.0)
int readAdcChannel(uint32_t channel);
int dacOutAdcIn(int dacValue, uint32_t adcChannel);
void getSnrDrvDacVal(void);
int bluetooth_report_index = 5;
int mvOffset = 100;
uint8_t txOverUsbUart = 1;

float normVout = 1.5;
float prevVout = -1.0;
float currVout = 0.0;
float diffVout = 0.0;

void checkUartBuf(void);
void CheckUartBuffer_Delay(uint32_t ms);

HAL_StatusTypeDef writeEEPRom32(uint32_t address, uint32_t data);
uint32_t readEEPROM32(uint32_t address);

#define USB_COM_BUFSIZE 128
uint8_t usbComOutBuf[USB_COM_BUFSIZE];
int sendOneTstResult(int dacVal, uint16_t adcVal);
int sendFiveTstResult(int * dacValArray, uint16_t * adcValArray);
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
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
	
	normVout = readEEPROM32(EEPROM_ADDR_NORMVOUT) / 1000.0;
	if ((normVout < 0) || (normVout > 3.0))
	{
		normVout = 1.5;
	}
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
			HAL_Delay(100);
		}
				
		if (wakeupSleepState == STATE_WAKEUP)
		{
			mvOffset = readEEPROM32(EEPROM_ADDR_OFFSET);
			if ((mvOffset <= 0) || (mvOffset > 1000))
			{
				mvOffset = 100;
			}
			
			uint16_t snrAdcVal[6] = {0, 0, 0, 0, 0, 0};
			
			setBlueToothDevPwr(DEV_PWR_ON);
			HAL_Delay(100);
			setSensorDevPwr(DEV_PWR_ON);
			CheckUartBuffer_Delay(100);
			//setSysWakeUpLed(LED_ON);
			//setSysStsLed(LED_OFF);
			
			int snrDrvDacValTemp = readEEPROM32(EEPROM_ADDR_SNRDRVVAL);
			if ((snrDrvDacValTemp < 1) || (snrDrvDacValTemp > 0xFFF))
			{
				getSnrDrvDacVal();
				snrDrvDacValTemp = readEEPROM32(EEPROM_ADDR_SNRDRVVAL);
			}
			
			int snrDrvDacVal[6];
			snrDrvDacVal[0] = snrDrvDacValTemp + MilliVoltToAdcVal(50);
			snrDrvDacVal[1] = snrDrvDacValTemp + MilliVoltToAdcVal(100);
			snrDrvDacVal[2] = snrDrvDacValTemp + MilliVoltToAdcVal(150);
			snrDrvDacVal[3] = snrDrvDacValTemp + MilliVoltToAdcVal(200);
			snrDrvDacVal[4] = snrDrvDacValTemp;
			snrDrvDacVal[5] = snrDrvDacValTemp + MilliVoltToAdcVal(mvOffset);
			
			for (int i=0; i<6; i++)
			{
				if (snrDrvDacVal[i] > 0xfff)
				{
					snrDrvDacVal[i] = 0xfff;
				}
			}
			
			for (int i=0; i<6; i++)
			{
				snrAdcVal[i] = (uint16_t)dacOutAdcIn(snrDrvDacVal[i], ADC_CHANNEL_8);
			}
			if (txOverUsbUart == 1)
			{
				sendFiveTstResult(snrDrvDacVal, snrAdcVal);
			}

			/*
			if (snrAdcVal[0] > SNR_PASS_THRESHOLD)
			{
				setTestPassLed(LED_ON);
				setTestFailLed(LED_OFF);
			}
			else
			{
				setTestFailLed(LED_ON);
				setTestPassLed(LED_OFF);
			}
			*/
			
			if (hi2c1.State == HAL_I2C_STATE_READY)
			{
				//float snrVoltage = ADCValToVolt(snrAdcVal[bluetooth_report_index]);
				currVout = ADCValToVolt(snrAdcVal[bluetooth_report_index]);
				if (prevVout < 0.0)
				{
					prevVout = currVout;
				}
				else
				{
					diffVout = currVout - prevVout;
					prevVout = currVout;
					normVout -= diffVout;
					if (normVout < 0.0)
					{
						normVout = 0.0;
						
					}
					else if (normVout > 3.0)
					{
						normVout = 3.0;
					}
					
					setBleSig(BLESIG_HI);
					if (HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t *)&normVout, sizeof(float), 3000) == HAL_OK)
					{
						setSysStsLed(LED_ON);
						HAL_Delay(100);
						setSysStsLed(LED_OFF);
						CheckUartBuffer_Delay(100);
						setSysStsLed(LED_ON);
						HAL_Delay(100);
						setSysStsLed(LED_OFF);
					}
					else
					{
						setSysWakeUpLed(LED_ON);
						CheckUartBuffer_Delay(300);
						setSysWakeUpLed(LED_OFF);
					}
				}
			}
			else
			{
				setSysWakeUpLed(LED_ON);
				HAL_Delay(100);
				setSysWakeUpLed(LED_OFF);
				CheckUartBuffer_Delay(100);
				setSysWakeUpLed(LED_ON);
				HAL_Delay(100);
				setSysWakeUpLed(LED_OFF);
			}
			
			setTestFailLed(LED_OFF);
			setTestPassLed(LED_OFF);
			CheckUartBuffer_Delay(650);
			setBleSig(BLESIG_LO);
			CheckUartBuffer_Delay(650);
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __CRS_CLK_ENABLE();

  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_CALCULATE_RELOADVALUE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

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
		setSysWakeUpLed(LED_ON);
		HAL_Delay(1000);
		GPIO_PinState buttonState = checkPushButton();
		if (buttonState == BUTTON_PRESSED)
		{
			buttonFunction = BTNFUNC_STATECHN;
			setSysWakeUpLed(LED_OFF);
			HAL_Delay(1000);
			GPIO_PinState buttonState = checkPushButton();
			if (buttonState == BUTTON_PRESSED)
			{
				setSysWakeUpLed(LED_ON);
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
			setSysWakeUpLed(LED_ON);
			HAL_Delay(10);
			setSysWakeUpLed(LED_OFF);
			HAL_Delay(200);
			setSysWakeUpLed(LED_ON);
			HAL_Delay(10);
			setSysWakeUpLed(LED_OFF);
		}
	} while (adcVolBat < (0x0400));
}

GPIO_PinState checkUsbPower(void)
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}

void setSysWakeUpLed(GPIO_PinState state)	// Red
{
#ifdef USE_SWSIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);
#else
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, state);
#endif
}

void setSysStsLed(GPIO_PinState state) // Green
{
#ifdef USE_SWSIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, state);
#else
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, state);
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
		sendOneTstResult(dacStep, adcValue);
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

int sendOneTstResult(int dacVal, uint16_t adcVal)
{
	memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
	sprintf((char *)usbComOutBuf, "%1.5f,%d,%1.5f\n",
					ADCValToVolt(dacVal), adcVal, ADCValToVolt(adcVal));
	CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
	
	return 0;
}

int sendFiveTstResult(int * dacValArray, uint16_t * adcValArray)
{
	static long long counter = 0;
	
	memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
	sprintf((char *)usbComOutBuf, "%1.5f,%d,%1.5f,%1.5f,%d,%1.5f,%1.5f,%d,%1.5f,%1.5f,%d,%1.5f,%1.5f,%d,%1.5f,%lld\n",
					ADCValToVolt(dacValArray[0]), adcValArray[0], ADCValToVolt(adcValArray[0]),
					ADCValToVolt(dacValArray[1]), adcValArray[1], ADCValToVolt(adcValArray[1]),
					ADCValToVolt(dacValArray[2]), adcValArray[2], ADCValToVolt(adcValArray[2]),
					ADCValToVolt(dacValArray[3]), adcValArray[3], ADCValToVolt(adcValArray[3]),
					ADCValToVolt(dacValArray[4]), adcValArray[4], ADCValToVolt(adcValArray[4]), counter++);
	CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
	
	return 0;
}

void CheckUartBuffer_Delay(uint32_t ms)
{
	checkUartBuf();
	HAL_Delay(ms);
}

void checkUartBuf(void)
{
	uint8_t rxMsg[RXCHRBUFSIZE];
	if (getMsg(rxMsg, RXCHRBUFSIZE) > 0)
	{
		int newSetValue = 0;
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
			case 'f':
			case 'F':
				switch (rxMsg[1])
				{
					case 0:
						break;
					default:
						newSetValue = atoi((const char *)&(rxMsg[1]));
						if ((newSetValue >= 0) && (newSetValue < 1000) && (newSetValue != mvOffset))
						{
							mvOffset = newSetValue;
							writeEEPRom32(EEPROM_ADDR_OFFSET, (uint32_t)newSetValue);
						}
						break;
				}
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "Offset(mv) = %d\n", mvOffset);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
			case 'n':
			case 'N':
				switch (rxMsg[1])
				{
					case 0:
						break;
					default:
						newSetValue = atoi((const char *)&(rxMsg[1]));
						if ((newSetValue >= 0) && (newSetValue <= 3000))
						{
							normVout = newSetValue / 1000.0;
							writeEEPRom32(EEPROM_ADDR_NORMVOUT, (uint32_t)newSetValue);
						}
						break;
				}
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "NormVout(mv) = %d\n", (int)(normVout * 1000));
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
			default:
				memset(usbComOutBuf, 0, USB_COM_BUFSIZE);
				sprintf((char *)usbComOutBuf, "Command \'%c\' is not valid.\n", rxMsg[0]);
				CDC_Transmit_FS(usbComOutBuf, strlen((char *)usbComOutBuf));
				break;
		}
	}
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
