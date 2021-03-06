/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_adc.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

/* Connection handle variable. */
uint8_t connection_handle = 0;
bool connected = false;

#define NUM_MOTOR_CTRL_CH (4)
uint32_t timerCount[NUM_MOTOR_CTRL_CH] = { 0 };
/**************************************************************************//**
 * @brief TIMER1_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
  /* Clear flag for TIMER1 overflow interrupt */
  for (int ch=0; ch<NUM_MOTOR_CTRL_CH; ch++) {
	TIMER_CompareBufSet(TIMER1, ch, timerCount[ch]);
  }
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
}

const uint16_t ADC_MAX = 0xFFF;
static uint32_t adcValueNew = 0, adcValueOld = 0;
/**************************************************************************//**
 * @brief ADC0 IRQ handler.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Single or scan DVL trigger */
  if ((ADC0->IEN & ADC_IEN_SINGLE) && (ADC0->IF & ADC_IF_SINGLE))
  {
    /* Read SINGLEDATA will clear SINGLE IF flag */
	  adcValueNew = ADC_DataSingleGet(ADC0);
  }
}
/**
 * @brief  Main function
 */
int main(void)
{
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  /* Initialize peripherals */
  enter_DefaultMode_from_RESET();

  //GPIO_PinOutClear(gpioPortD, 15);
  const uint32_t PWM_FREQ = 16000;
  uint32_t pwmClkFreq = CMU_ClockFreqGet(cmuClock_HFPER);
  uint32_t topVal = pwmClkFreq/PWM_FREQ;

  /* Initialize stack */
  gecko_init(&config);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

        /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        gecko_cmd_le_gap_set_adv_parameters(160, 160, 7);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        break;

      // LE CONNECTION OPENED (remote device connected)
      case gecko_evt_le_connection_opened_id:
    	/* 1 second tick */
    	gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
    	{
          /* Enable ADC Interrupt when reaching DVL and window compare */
          ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
          /* Clear the FIFOs and pending interrupt */
          ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
          NVIC_ClearPendingIRQ(ADC0_IRQn);
          NVIC_EnableIRQ(ADC0_IRQn);
    	}

    	{
    	  /* Timer1 form 4 pwm channels */
    	  TIMER_TopSet(TIMER1, topVal);
    	  timerCount[0] = topVal * 3 / 4;
    	  timerCount[1] = topVal / 4;
    	  timerCount[2] = topVal * adcValueNew / ADC_MAX;
    	  timerCount[3] = topVal * (ADC_MAX - adcValueNew) / ADC_MAX;
    	  /* Enable timer1 interrupt */
    	  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
    	  /* Enable TIMER1 interrupt vector in NVIC */
    	  NVIC_EnableIRQ(TIMER1_IRQn);
    	}
        connection_handle = evt->data.evt_le_connection_opened.connection;
        connected = true;
        break;

      case gecko_evt_le_connection_closed_id:

        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        }

    	gecko_cmd_hardware_set_soft_timer(0, 0, 0);
    	ADC_IntDisable(ADC0, ADC_IEN_SINGLE);
        NVIC_DisableIRQ(ADC0_IRQn);
    	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
        for (int ch=0; ch<NUM_MOTOR_CTRL_CH; ch++) {
          timerCount[ch] = 0;
          TIMER_CompareBufSet(TIMER1, ch, timerCount[ch]);
        }
    	NVIC_DisableIRQ(TIMER1_IRQn);
        connected = false;
        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_hardware_soft_timer_id:
        /* Measure the ADC */
        //if (connected) {
          if (adcValueOld != adcValueNew) {
	        adcValueOld = adcValueNew;
	        timerCount[2] = topVal * adcValueNew / ADC_MAX;
	        uint8_t batVol = 50 * adcValueNew / ADC_MAX;
	        gecko_cmd_gatt_server_write_attribute_value(gattdb_batVol, 0, 1, &batVol);
	        gecko_cmd_gatt_server_send_characteristic_notification(connection_handle, gattdb_batVol, 1, &batVol);
          }
      	  ADC_Start(ADC0, adcStartSingle);
        //}
        break;

      // GATT SERVER USER WRITE REQUEST (remote GATT client wrote a new value to a user-type characteristic)
      case gecko_evt_gatt_server_user_write_request_id:
        /* Check if the user-type OTA Control Characteristic was written.
         * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
        } else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_pwm1Percentage) {
          if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
          {
        	uint8_t percent = evt->data.evt_gatt_server_user_write_request.value.data[0];
            timerCount[0] = topVal * (percent <= 100 ? percent : 0) / 100;
			gecko_cmd_gatt_server_send_user_write_response(
				evt->data.evt_gatt_server_user_write_request.connection,
				evt->data.evt_gatt_server_user_write_request.characteristic,
				0x00 /* SUCCESS */);
          }
        } else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_pwm2Percentage) {
          if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
          {
        	uint8_t percent = evt->data.evt_gatt_server_user_write_request.value.data[0];
            timerCount[1] = topVal * (percent <= 100 ? percent : 0) / 100;
			gecko_cmd_gatt_server_send_user_write_response(
				evt->data.evt_gatt_server_user_write_request.connection,
				evt->data.evt_gatt_server_user_write_request.characteristic,
				0x00 /* SUCCESS */);
          }
        } else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_pwm3Percentage) {
          if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
          {
        	//uint8_t percent = evt->data.evt_gatt_server_user_write_request.value.data[0];
            //timerCount[2] = topVal * (percent <= 100 ? percent : 0) / 100;
			gecko_cmd_gatt_server_send_user_write_response(
				evt->data.evt_gatt_server_user_write_request.connection,
				evt->data.evt_gatt_server_user_write_request.characteristic,
				0x00 /* SUCCESS */);
          }
        } else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_pwm4Percentage) {
          if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
          {
        	uint8_t percent = evt->data.evt_gatt_server_user_write_request.value.data[0];
            timerCount[3] = topVal * (percent <= 100 ? percent : 0) / 100;
			gecko_cmd_gatt_server_send_user_write_response(
				evt->data.evt_gatt_server_user_write_request.connection,
				evt->data.evt_gatt_server_user_write_request.characteristic,
				0x00 /* SUCCESS */);
          }
        }
        break;

      default:
        break;
    }
  }
  return 0;
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
