/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_at.c
  * @author  MCD Application Team
  * @brief   AT command API
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <strings.h>
#include "platform.h"
#include "lora_at.h"
#include "sys_app.h"
#include "stm32_tiny_sscanf.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "test_rf.h"
#include "adc_if.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "radio.h"
#include "lora_info.h"
#include "flash_if.h"
#include "eeprom_wrapper.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "LoRaMac.h"
#include "LoRaMacCrypto.h"
#include "Region.h"
#include "lora_app.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static bool ClassBEnableRequest = false;

/*!
 * User application buffer
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/* Dummy data sent periodically to let the tester respond with start test command*/
static UTIL_TIMER_Object_t TxCertifTimer;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Get 4 bytes values in hex
  * @param  from string containing the 16 bytes, something like ab:cd:01:21
  * @param  value buffer that will contain the bytes read
  * @retval The number of bytes read
  */
static int32_t sscanf_uint32_as_hhx(const char *from, uint32_t *value);

/**
  * @brief  Get 16 bytes values in hex
  * @param  from string containing the 16 bytes, something like ab:cd:01:...
  * @param  pt buffer that will contain the bytes read
  * @retval The number of bytes read
  */
static int32_t sscanf_16_hhx(const char *from, uint8_t *pt);

/**
  * @brief  Print 4 bytes as %02x
  * @param  value containing the 4 bytes to print
  */
static void print_uint32_as_02x(uint32_t value);

/**
  * @brief  Print 16 bytes as %02X
  * @param  pt pointer containing the 16 bytes to print
  */
static void print_16_02x(uint8_t *pt);

/**
  * @brief  Print 8 bytes as %02X
  * @param  pt pointer containing the 8 bytes to print
  */
static void print_8_02x(uint8_t *pt);

/**
  * @brief  Print an int
  * @param  value to print
  */
static void print_d(int32_t value);

/**
  * @brief  Print an unsigned int
  * @param  value to print
  */
static void print_u(uint32_t value);

/**
  * @brief  Certif Rejoin timer callback function
  * @param  context ptr of Certif Rejoin context
  */
static void OnCertifTimer(void *context);

/**
  * @brief  Certif send function
  */
static void CertifSend(void);

/**
  * @brief  Check if character in parameter is alphanumeric
  * @param  Char for the alphanumeric check
  */
static int32_t isHex(char Char);

/**
  * @brief  Converts hex string to a nibble ( 0x0X )
  * @param  Char hex string
  * @retval the nibble. Returns 0xF0 in case input was not an hex string (a..f; A..F or 0..9)
  */
static uint8_t Char2Nibble(char Char);

/**
  * @brief  Convert a string into a buffer of data
  * @param  str string to convert
  * @param  data output buffer
  * @param  Size of input string
  */
static int32_t stringToData(const char *str, uint8_t *data, uint32_t Size);

static void AT_flush_output(void)
{
  while (UTIL_ADV_TRACE_IsBufferEmpty() == 0U)
  {
  }
  HAL_Delay(2);        /* 1 byte at 9600 Bd = 1.04 ms */
}
/* USER CODE BEGIN PFP */
static ATEerror_t AT_nvm_store(uint8_t force);
/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
ATEerror_t AT_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t AT_return_error(const char *param)
{
  return AT_ERROR;
}

/* --------------- Application events --------------- */
ATEerror_t AT_run(){
	AT_PPRINTF("+AT: OK\r\n");      /* Seeed format; the ESP driver compares it exactly (trim + strcmp) */
	return AT_OK;
}

void AT_event_join(LmHandlerJoinParams_t *params)
{
  /* USER CODE BEGIN AT_event_join_1 */
  return;   /* Seeed emulation: no +EVT lines, they break get_modem_answer() of the ESP driver */
  /* USER CODE END AT_event_join_1 */
  if ((params != NULL) && (params->Status == LORAMAC_HANDLER_SUCCESS))
  {
    AT_PPRINTF("+EVT:JOINED\r\n");
  }
  else
  {
    AT_PPRINTF("+EVT:JOIN FAILED\r\n");
  }
  /* USER CODE BEGIN AT_event_join_2 */

  /* USER CODE END AT_event_join_2 */
}

void AT_event_receive(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN AT_event_receive_1 */
  //return;   /* Seeed emulation: no +EVT lines (they came after +CMSGHEX: Done, breaking get_modem_answer()) */
  /* USER CODE END AT_event_receive_1 */
  const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
  uint8_t ReceivedDataSize = 0;
  uint8_t RxPort = 0;

  if (appData != NULL)
  {
    RxPort = appData->Port;
    if ((appData->Buffer != NULL) && (appData->BufferSize > 0))
    {
      /* Received data to be copied*/
      if (LORAWAN_APP_DATA_BUFFER_MAX_SIZE <= appData->BufferSize)
      {
        ReceivedDataSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE;
      }
      else
      {
        ReceivedDataSize = appData->BufferSize;
      }

      /*asynchronous notification to the host*/
      AT_PRINTF("+EVT:%d:%02X:", appData->Port, ReceivedDataSize);

      for (uint8_t i = 0; i < ReceivedDataSize; i++)
      {
        AT_PRINTF("%02X", appData->Buffer[i]);
      }
      AT_PRINTF("\r\n");
    }
  }

  if ((params != NULL) && (params->RxSlot < RX_SLOT_NONE))
  {
    AT_PRINTF("+EVT:RX_%s, PORT %d, DR %d, RSSI %d, SNR %d", slotStrings[params->RxSlot], RxPort,
              params->Datarate, params->Rssi, params->Snr);
    if (params->LinkCheck == true)
    {
      AT_PRINTF(", DMODM %d, GWN %d", params->DemodMargin, params->NbGateways);
    }
    AT_PRINTF("\r\n");
  }

  /* USER CODE BEGIN AT_event_receive_2 */

  /* USER CODE END AT_event_receive_2 */
}

void AT_event_confirm(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN AT_event_confirm_1 */
  /* AT+MSGHEX/CMSGHEX end (NbTrans restore, +..: Done); Seeed emulation: no +EVT:SEND_CONFIRMED */
  if (params != NULL)
  {
    AT_msghex_done(params);
  }
  return;
  /* USER CODE END AT_event_confirm_1 */
  if ((params != NULL) && (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG) && (params->AckReceived != 0))
  {
    AT_PRINTF("+EVT:SEND_CONFIRMED\r\n");
  }
  /* USER CODE BEGIN AT_event_confirm_2 */

  /* USER CODE END AT_event_confirm_2 */
}

void AT_event_ClassUpdate(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN AT_event_ClassUpdate_1 */
  return;   /* Seeed emulation: no +EVT lines */
  /* USER CODE END AT_event_ClassUpdate_1 */
  AT_PRINTF("+EVT:SWITCH_TO_CLASS_%c\r\n", "ABC"[deviceClass]);
  /* USER CODE BEGIN AT_event_ClassUpdate_2 */

  /* USER CODE END AT_event_ClassUpdate_2 */
}

void AT_event_Beacon(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN AT_event_Beacon_1 */
  return;   /* Seeed emulation: no +EVT lines */
  /* USER CODE END AT_event_Beacon_1 */
  if (params != NULL)
  {
    switch (params->State)
    {
      default:
      case LORAMAC_HANDLER_BEACON_LOST:
      {
        AT_PRINTF("+EVT:BEACON_LOST\r\n");
        break;
      }
      case LORAMAC_HANDLER_BEACON_RX:
      {
        AT_PRINTF("+EVT:RX_BC, DR %d, RSSI %d, SNR %d, FQ %d, TIME %d, DESC %d, "
                  "INFO %02X%02X%02X,%02X%02X%02X\r\n",
                  params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
                  params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
                  params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
                  params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
                  params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
        break;
      }
      case LORAMAC_HANDLER_BEACON_NRX:
      {
        AT_PRINTF("+EVT:BEACON_NOT_RECEIVED\r\n");
        break;
      }
    }
  }
  /* USER CODE BEGIN AT_event_Beacon_2 */

  /* USER CODE END AT_event_Beacon_2 */
}

void AT_event_OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* debug log only (AT+LOG=DEBUG): an unsolicited line would break get_modem_answer() of the ESP driver */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
}

void AT_event_OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
}

void AT_event_OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
}

/* --------------- General commands --------------- */
ATEerror_t AT_version_get(const char *param)
{
  /* USER CODE BEGIN AT_version_get_1 */

  /* USER CODE END AT_version_get_1 */
  uint32_t feature_version;

  /* Get LoRa APP version*/
  AT_PRINTF("APPLICATION_VERSION: V%X.%X.%X\r\n",
            (uint8_t)(APP_VERSION_MAIN),
            (uint8_t)(APP_VERSION_SUB1),
            (uint8_t)(APP_VERSION_SUB2));

  /* Get MW LoraWAN info */
  AT_PRINTF("MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
            (uint8_t)(LORAWAN_VERSION_MAIN),
            (uint8_t)(LORAWAN_VERSION_SUB1),
            (uint8_t)(LORAWAN_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  AT_PRINTF("MW_RADIO_VERSION:    V%X.%X.%X\r\n",
            (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
            (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
            (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Get LoraWAN Link Layer info */
  LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
  AT_PRINTF("L2_SPEC_VERSION:     V%X.%X.%X\r\n",
            (uint8_t)(feature_version >> 24),
            (uint8_t)(feature_version >> 16),
            (uint8_t)(feature_version >> 8));

  /* Get LoraWAN Regional Parameters info */
  LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
  AT_PRINTF("RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
            (uint8_t)(feature_version >> 24),
            (uint8_t)(feature_version >> 16),
            (uint8_t)(feature_version >> 8),
            (uint8_t)(feature_version));

  return AT_OK;
  /* USER CODE BEGIN AT_version_get_2 */

  /* USER CODE END AT_version_get_2 */
}

ATEerror_t AT_verbose_get(const char *param)
{
  /* USER CODE BEGIN AT_verbose_get_1 */

  /* USER CODE END AT_verbose_get_1 */
  print_u(UTIL_ADV_TRACE_GetVerboseLevel());
  return AT_OK;
  /* USER CODE BEGIN AT_verbose_get_2 */

  /* USER CODE END AT_verbose_get_2 */
}

ATEerror_t AT_verbose_set(const char *param)
{
  /* USER CODE BEGIN AT_verbose_set_1 */

  /* USER CODE END AT_verbose_set_1 */
  const char *buf = param;
  int32_t lvl_nb;

  /* read and set the verbose level */
  if (1 != tiny_sscanf(buf, "%u", &lvl_nb))
  {
    AT_PRINTF("AT+VL: verbose level is not well set\r\n");
    return AT_PARAM_ERROR;
  }
  if ((lvl_nb > VLEVEL_H) || (lvl_nb < VLEVEL_OFF))
  {
    AT_PRINTF("AT+VL: verbose level out of range => 0(VLEVEL_OFF) to 3(VLEVEL_H)\r\n");
    return AT_PARAM_ERROR;
  }

  UTIL_ADV_TRACE_SetVerboseLevel(lvl_nb);

  return AT_OK;
  /* USER CODE BEGIN AT_verbose_set_2 */

  /* USER CODE END AT_verbose_set_2 */
}

ATEerror_t AT_LocalTime_get(const char *param)
{
  /* USER CODE BEGIN AT_LocalTime_get_1 */

  /* USER CODE END AT_LocalTime_get_1 */
  struct tm localtime;
  SysTime_t UnixEpoch = SysTimeGet();
  UnixEpoch.Seconds -= 18; /*removing leap seconds*/

  UnixEpoch.Seconds += 3600 * 2; /*adding 2 hours*/

  SysTimeLocalTime(UnixEpoch.Seconds,  & localtime);

  AT_PRINTF("LTIME:%02dh%02dm%02ds on %02d/%02d/%04d\r\n",
            localtime.tm_hour, localtime.tm_min, localtime.tm_sec,
            localtime.tm_mday, localtime.tm_mon + 1, localtime.tm_year + 1900);

  return AT_OK;
  /* USER CODE BEGIN AT_LocalTime_get_2 */

  /* USER CODE END AT_LocalTime_get_2 */
}

ATEerror_t AT_reset(const char *param)
{
  /* USER CODE BEGIN AT_reset_1 */
  (void)AT_nvm_store(0U);     /* changed configuration must not be lost by the reset (AT+UART=BR -> AT+RESET) */
  AT_PPRINTF("+RESET: OK\r\n");
  AT_flush_output();      /* the answer must leave the UART before the reset (ESP waits for it before the baud change) */
  NVIC_SystemReset();
  /* USER CODE END AT_reset_1 */
  /* USER CODE BEGIN AT_reset_2 */

  /* USER CODE END AT_reset_2 */
}

/* ---------------  Context Store commands --------------------------- */
ATEerror_t AT_restore_factory_settings(const char *param)
{
  /* USER CODE BEGIN AT_restore_factory_settings_1 */

  /* USER CODE END AT_restore_factory_settings_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    /* System Reboot*/
    NVIC_SystemReset();
  }

  return AT_OK;
  /* USER CODE BEGIN AT_restore_factory_settings_2 */

  /* USER CODE END AT_restore_factory_settings_2 */
}

ATEerror_t AT_store_context(const char *param)
{
  /* USER CODE BEGIN AT_store_context_1 */

  /* USER CODE END AT_store_context_1 */
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    AT_PRINTF("NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    AT_PRINTF("NVM DATA STORE FAILED\r\n");
    return AT_ERROR;
  }
  return AT_OK;
  /* USER CODE BEGIN AT_store_context_2 */

  /* USER CODE END AT_store_context_2 */
}

/* --------------- Keys, IDs and EUIs management commands --------------- */
ATEerror_t AT_JoinEUI_get(const char *param)
{
  /* USER CODE BEGIN AT_JoinEUI_get_1 */

  /* USER CODE END AT_JoinEUI_get_1 */
  uint8_t appEUI[8];
  if (LmHandlerGetAppEUI(appEUI) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_8_02x(appEUI);
  return AT_OK;
  /* USER CODE BEGIN AT_JoinEUI_get_2 */

  /* USER CODE END AT_JoinEUI_get_2 */
}

ATEerror_t AT_JoinEUI_set(const char *param)
{
  /* USER CODE BEGIN AT_JoinEUI_set_1 */

  /* USER CODE END AT_JoinEUI_set_1 */
  uint8_t JoinEui[8];
  if (tiny_sscanf(param, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                  &JoinEui[0], &JoinEui[1], &JoinEui[2], &JoinEui[3],
                  &JoinEui[4], &JoinEui[5], &JoinEui[6], &JoinEui[7]) != 8)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetAppEUI(JoinEui))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_JoinEUI_set_2 */

  /* USER CODE END AT_JoinEUI_set_2 */
}

ATEerror_t AT_NwkKey_get(const char *param)
{
  /* USER CODE BEGIN AT_NwkKey_get_1 */

  /* USER CODE END AT_NwkKey_get_1 */
  uint8_t nwkKey[16];
  if (LORAMAC_HANDLER_SUCCESS != LmHandlerGetKey(NWK_KEY, nwkKey))
  {
    return AT_ERROR;
  }
  print_16_02x(nwkKey);

  return AT_OK;
  /* USER CODE BEGIN AT_NwkKey_get_2 */

  /* USER CODE END AT_NwkKey_get_2 */
}

ATEerror_t AT_NwkKey_set(const char *param)
{
  /* USER CODE BEGIN AT_NwkKey_set_1 */

  /* USER CODE END AT_NwkKey_set_1 */
  uint8_t nwkKey[16];
  if (sscanf_16_hhx(param, nwkKey) != 16)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetKey(NWK_KEY, nwkKey))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_NwkKey_set_2 */

  /* USER CODE END AT_NwkKey_set_2 */
}

ATEerror_t AT_AppKey_get(const char *param)
{
  /* USER CODE BEGIN AT_AppKey_get_1 */

  /* USER CODE END AT_AppKey_get_1 */
  uint8_t appKey[16];
  if (LORAMAC_HANDLER_SUCCESS != LmHandlerGetKey(APP_KEY, appKey))
  {
    return AT_ERROR;
  }
  print_16_02x(appKey);

  return AT_OK;
  /* USER CODE BEGIN AT_AppKey_get_2 */

  /* USER CODE END AT_AppKey_get_2 */
}

ATEerror_t AT_AppKey_set(const char *param)
{
  /* USER CODE BEGIN AT_AppKey_set_1 */

  /* USER CODE END AT_AppKey_set_1 */
  uint8_t appKey[16];
  if (sscanf_16_hhx(param, appKey) != 16)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetKey(APP_KEY, appKey))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_AppKey_set_2 */

  /* USER CODE END AT_AppKey_set_2 */
}

ATEerror_t AT_NwkSKey_get(const char *param)
{
  /* USER CODE BEGIN AT_NwkSKey_get_1 */

  /* USER CODE END AT_NwkSKey_get_1 */
  uint8_t nwkSKey[16];
  if (LORAMAC_HANDLER_SUCCESS != LmHandlerGetKey(NWK_S_KEY, nwkSKey))
  {
    return AT_ERROR;
  }
  print_16_02x(nwkSKey);

  return AT_OK;
  /* USER CODE BEGIN AT_NwkSKey_get_2 */

  /* USER CODE END AT_NwkSKey_get_2 */
}

ATEerror_t AT_NwkSKey_set(const char *param)
{
  /* USER CODE BEGIN AT_NwkSKey_set_1 */

  /* USER CODE END AT_NwkSKey_set_1 */
  uint8_t nwkSKey[16];
  if (sscanf_16_hhx(param, nwkSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetKey(NWK_S_KEY, nwkSKey))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_NwkSKey_set_2 */

  /* USER CODE END AT_NwkSKey_set_2 */
}

ATEerror_t AT_AppSKey_get(const char *param)
{
  /* USER CODE BEGIN AT_AppSKey_get_1 */

  /* USER CODE END AT_AppSKey_get_1 */
  uint8_t appSKey[16];
  if (LORAMAC_HANDLER_SUCCESS != LmHandlerGetKey(APP_S_KEY, appSKey))
  {
    return AT_ERROR;
  }
  print_16_02x(appSKey);

  return AT_OK;
  /* USER CODE BEGIN AT_AppSKey_get_2 */

  /* USER CODE END AT_AppSKey_get_2 */
}

ATEerror_t AT_AppSKey_set(const char *param)
{
  /* USER CODE BEGIN AT_AppSKey_set_1 */

  /* USER CODE END AT_AppSKey_set_1 */
  uint8_t appSKey[16];
  if (sscanf_16_hhx(param, appSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetKey(APP_S_KEY, appSKey))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_AppSKey_set_2 */

  /* USER CODE END AT_AppSKey_set_2 */
}

ATEerror_t AT_DevAddr_get(const char *param)
{
  /* USER CODE BEGIN AT_DevAddr_get_1 */

  /* USER CODE END AT_DevAddr_get_1 */
  uint32_t devAddr;
  if (LmHandlerGetDevAddr(&devAddr) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_uint32_as_02x(devAddr);
  return AT_OK;
  /* USER CODE BEGIN AT_DevAddr_get_2 */

  /* USER CODE END AT_DevAddr_get_2 */
}

ATEerror_t AT_DevAddr_set(const char *param)
{
  /* USER CODE BEGIN AT_DevAddr_set_1 */

  /* USER CODE END AT_DevAddr_set_1 */
  uint32_t devAddr;
  if (sscanf_uint32_as_hhx(param, &devAddr) != 4)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetDevAddr(devAddr))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_DevAddr_set_2 */

  /* USER CODE END AT_DevAddr_set_2 */
}

ATEerror_t AT_DevEUI_get(const char *param)
{
  /* USER CODE BEGIN AT_DevEUI_get_1 */

  /* USER CODE END AT_DevEUI_get_1 */
  uint8_t devEUI[8];
  if (LmHandlerGetDevEUI(devEUI) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_8_02x(devEUI);
  return AT_OK;
  /* USER CODE BEGIN AT_DevEUI_get_2 */

  /* USER CODE END AT_DevEUI_get_2 */
}

ATEerror_t AT_DevEUI_set(const char *param)
{
  /* USER CODE BEGIN AT_DevEUI_set_1 */

  /* USER CODE END AT_DevEUI_set_1 */
  uint8_t devEui[8];
  if (tiny_sscanf(param, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                  &devEui[0], &devEui[1], &devEui[2], &devEui[3],
                  &devEui[4], &devEui[5], &devEui[6], &devEui[7]) != 8)
  {
    return AT_PARAM_ERROR;
  }

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerSetDevEUI(devEui))
  {
    return AT_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_DevEUI_set_2 */

  /* USER CODE END AT_DevEUI_set_2 */
}

ATEerror_t AT_NetworkID_get(const char *param)
{
  /* USER CODE BEGIN AT_NetworkID_get_1 */

  /* USER CODE END AT_NetworkID_get_1 */
  uint32_t networkId;
  if (LmHandlerGetNetworkID(&networkId) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(networkId);
  return AT_OK;
  /* USER CODE BEGIN AT_NetworkID_get_2 */

  /* USER CODE END AT_NetworkID_get_2 */
}

ATEerror_t AT_NetworkID_set(const char *param)
{
  /* USER CODE BEGIN AT_NetworkID_set_1 */

  /* USER CODE END AT_NetworkID_set_1 */
  uint32_t networkId;
  if (tiny_sscanf(param, "%u", &networkId) != 1)
  {
    return AT_PARAM_ERROR;
  }

  if (networkId > 127)
  {
    return AT_PARAM_ERROR;
  }

  LmHandlerSetNetworkID(networkId);
  return AT_OK;
  /* USER CODE BEGIN AT_NetworkID_set_2 */

  /* USER CODE END AT_NetworkID_set_2 */
}

/* --------------- LoRaWAN join and send data commands --------------- */
ATEerror_t AT_Join(const char *param)
{
  /* USER CODE BEGIN AT_Join_1 */

  /* USER CODE END AT_Join_1 */
  switch (param[0])
  {
    case '0':
      LmHandlerJoin(ACTIVATION_TYPE_ABP, true);
      break;
    case '1':
      LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Join_2 */

  /* USER CODE END AT_Join_2 */
}

ATEerror_t AT_Link_Check(const char *param)
{
  /* USER CODE BEGIN AT_Link_Check_1 */

  /* USER CODE END AT_Link_Check_1 */
  if (LmHandlerLinkCheckReq() != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Link_Check_2 */

  /* USER CODE END AT_Link_Check_2 */
}

ATEerror_t AT_Send(const char *param)
{
  /* USER CODE BEGIN AT_Send_1 */

  /* USER CODE END AT_Send_1 */
  const char *buf = param;
  uint16_t bufSize = strlen(param);
  uint32_t appPort;
  LmHandlerMsgTypes_t isTxConfirmed;
  unsigned size = 0;
  char hex[3] = {0, 0, 0};
  LmHandlerErrorStatus_t lmhStatus = LORAMAC_HANDLER_ERROR;
  ATEerror_t status = AT_ERROR;

  /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    AT_PRINTF("AT+SEND without the application port\r\n");
    return AT_PARAM_ERROR;
  }

  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9') && bufSize > 1)
  {
    buf ++;
    bufSize --;
  };

  if ((bufSize == 0) || (':' != buf[0]))
  {
    AT_PRINTF("AT+SEND missing : character after app port\r\n");
    return AT_PARAM_ERROR;
  }
  else
  {
    /* skip the char ':' */
    buf ++;
    bufSize --;
  }

  switch (buf[0])
  {
    case '0':
      isTxConfirmed = LORAMAC_HANDLER_UNCONFIRMED_MSG;
      break;
    case '1':
      isTxConfirmed = LORAMAC_HANDLER_CONFIRMED_MSG;
      break;
    default:
      AT_PRINTF("AT+SEND without the acknowledge flag\r\n");
      return AT_PARAM_ERROR;
  }

  if (bufSize > 0)
  {
    /* skip the acknowledge flag */
    buf ++;
    bufSize --;
  }

  if ((bufSize == 0) || (':' != buf[0]))
  {
    AT_PRINTF("AT+SEND missing : character after ack flag\r\n");
    return AT_PARAM_ERROR;
  }
  else
  {
    /* skip the char ':' */
    buf ++;
    bufSize --;
  }

  while ((size < LORAWAN_APP_DATA_BUFFER_MAX_SIZE) && (bufSize > 1))
  {
    hex[0] = buf[size * 2];
    hex[1] = buf[size * 2 + 1];
    if (tiny_sscanf(hex, "%hhx", &AppData.Buffer[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }

  AppData.BufferSize = size;
  AppData.Port = appPort;

  lmhStatus = LmHandlerSend(&AppData, isTxConfirmed, false);

  switch (lmhStatus)
  {
    case LORAMAC_HANDLER_SUCCESS:
      status = AT_OK;
      break;
    case LORAMAC_HANDLER_BUSY_ERROR:
    case LORAMAC_HANDLER_COMPLIANCE_RUNNING:
      status = (LmHandlerJoinStatus() != LORAMAC_HANDLER_SET) ? AT_NO_NET_JOINED : AT_BUSY_ERROR;
      break;
    case LORAMAC_HANDLER_NO_NETWORK_JOINED:
      status = AT_NO_NET_JOINED;
      break;
    case LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED:
      status = AT_DUTYCYCLE_RESTRICTED;
      break;
    case LORAMAC_HANDLER_CRYPTO_ERROR:
      status = AT_CRYPTO_ERROR;
      break;
    case LORAMAC_HANDLER_ERROR:
    default:
      status = AT_ERROR;
      break;
  }

  return status;
  /* USER CODE BEGIN AT_Send_2 */

  /* USER CODE END AT_Send_2 */
}

/* --------------- LoRaWAN network management commands --------------- */
ATEerror_t AT_ADR_get(const char *param)
{
  /* USER CODE BEGIN AT_ADR_get_1 */

  /* USER CODE END AT_ADR_get_1 */
  bool adrEnable;
  if (LmHandlerGetAdrEnable(&adrEnable) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(adrEnable);
  return AT_OK;
  /* USER CODE BEGIN AT_ADR_get_2 */

  /* USER CODE END AT_ADR_get_2 */
}

ATEerror_t AT_ADR_set(const char *param){
	bool adr = false;
	if (strcasecmp(param, "ON") == 0){
		adr = true;
	}
	else if (strcasecmp(param, "OFF") == 0){
		adr = false;
	}
	else{
		return AT_PARAM_ERROR;
	}

	if (LmHandlerSetAdrEnable(adr) != LORAMAC_HANDLER_SUCCESS){
		return AT_ERROR;
	}
	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+ADR: %s\r\n", adr == true ? "ON" : "OFF");
	return AT_OK;
}

ATEerror_t AT_DataRate_get(const char *param)
{
  /* USER CODE BEGIN AT_DataRate_get_1 */

  /* USER CODE END AT_DataRate_get_1 */
  int8_t txDatarate;
  if (LmHandlerGetTxDatarate(&txDatarate) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(txDatarate);
  return AT_OK;
  /* USER CODE BEGIN AT_DataRate_get_2 */

  /* USER CODE END AT_DataRate_get_2 */
}

ATEerror_t AT_DataRate_set(const char *param){
	char *end;
	unsigned long dr;
	uint8_t datarate;
	LoRaMacRegion_t region;
	bool adr;
	LmHandlerErrorStatus_t status;

	if (strcasecmp(param, "EU868") == 0){
		/* Single region build: EU868 is always active, only checked. LmHandlerSetActiveRegion() refuses
		   after the ABP activation (LmHandler.c) and re-runs LmHandlerConfigure() before it */
		if ((LmHandlerGetActiveRegion(&region) != LORAMAC_HANDLER_SUCCESS) || (region != LORAMAC_REGION_EU868)){
			return AT_ERROR;
		}
		AT_PPRINTF("+DR: EU868\r\n");
		return AT_OK;
	}
	/* DR0..DR5 = SF12..SF7 at 125 kHz (DR6 = SF7/BW250, DR7 = FSK: not used) */
	dr = strtoul(param, &end, 10);
	if ((end == param) || (*end != '\0') || (dr > 5UL)){
		return AT_PARAM_ERROR;
	}
	datarate = (uint8_t)dr;
	LmHandlerGetAdrEnable(&adr);
	if (adr){
		LmHandlerSetAdrEnable(false);
	}
	status = LmHandlerSetTxDatarate(datarate);
	if (adr){
		LmHandlerSetAdrEnable(true);
	}
	if (status != LORAMAC_HANDLER_SUCCESS){
		return AT_ERROR;
	}
	if (eeprom_set_dr((uint8_t)datarate) != EE_OK){
		return AT_ERROR;
	}
	/* EU868 only (single region build) */
	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+DR: EU868 DR%u  SF%u%sBW125K\r\n", (unsigned)datarate, (unsigned)(12U - datarate),
			(datarate > 2U) ? "  " : " ");
	return AT_OK;
}

ATEerror_t AT_Region_get(const char *param)
{
  /* USER CODE BEGIN AT_Region_get_1 */

  /* USER CODE END AT_Region_get_1 */
  const char *regionStrings[] =
  {
    "AS923", "AU915", "CN470", "CN779", "EU433", "EU868", "KR920", "IN865", "US915", "RU864"
  };
  LoRaMacRegion_t region;
  if (LmHandlerGetActiveRegion(&region) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  if (region > LORAMAC_REGION_RU864)
  {
    return AT_PARAM_ERROR;
  }

  AT_PRINTF("%d:%s\r\n", region, regionStrings[region]);
  return AT_OK;
  /* USER CODE BEGIN AT_Region_get_2 */

  /* USER CODE END AT_Region_get_2 */
}

ATEerror_t AT_Region_set(const char *param)
{
  /* USER CODE BEGIN AT_Region_set_1 */

  /* USER CODE END AT_Region_set_1 */
  LoRaMacRegion_t region;
  if (tiny_sscanf(param, "%hhu", &region) != 1)
  {
    return AT_PARAM_ERROR;
  }
  if (region > LORAMAC_REGION_RU864)
  {
    return AT_PARAM_ERROR;
  }

  if (LmHandlerSetActiveRegion(region) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Region_set_2 */

  /* USER CODE END AT_Region_set_2 */
}

ATEerror_t AT_DeviceClass_get(const char *param)
{
  /* USER CODE BEGIN AT_DeviceClass_get_1 */

  /* USER CODE END AT_DeviceClass_get_1 */
  DeviceClass_t currentClass;
  LoraInfo_t *loraInfo = LoraInfo_GetPtr();
  if (loraInfo == NULL)
  {
    return AT_ERROR;
  }

  if (LmHandlerGetCurrentClass(&currentClass) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  if ((loraInfo->ClassB == 1) && (ClassBEnableRequest == true) && (currentClass == CLASS_A))
  {
    BeaconState_t beaconState;

    if (LmHandlerGetBeaconState(&beaconState) != LORAMAC_HANDLER_SUCCESS)
    {
      return AT_PARAM_ERROR;
    }

    if ((beaconState == BEACON_STATE_ACQUISITION) ||
        (beaconState == BEACON_STATE_ACQUISITION_BY_TIME) ||
        (beaconState == BEACON_STATE_REACQUISITION)) /*Beacon_Searching on Class B request*/
    {
      AT_PRINTF("B,S0\r\n");
    }
    else if ((beaconState == BEACON_STATE_LOCKED) || /*Beacon locked on Gateway*/
             (beaconState == BEACON_STATE_IDLE)   ||
             (beaconState == BEACON_STATE_GUARD)  ||
             (beaconState == BEACON_STATE_RX))
    {
      AT_PRINTF("B,S1\r\n");
    }
    else
    {
      AT_PRINTF("B,S2\r\n");
    }
  }
  else /* we are now either in Class B enable or Class C enable*/
  {
    AT_PRINTF("%c\r\n", 'A' + currentClass);
  }

  return AT_OK;
  /* USER CODE BEGIN AT_DeviceClass_get_2 */

  /* USER CODE END AT_DeviceClass_get_2 */
}

ATEerror_t AT_DeviceClass_set(const char *param)
{
  /* USER CODE BEGIN AT_DeviceClass_set_1 */

  /* USER CODE END AT_DeviceClass_set_1 */
  LmHandlerErrorStatus_t errorStatus = LORAMAC_HANDLER_SUCCESS;
  LoraInfo_t *loraInfo = LoraInfo_GetPtr();
  if (loraInfo == NULL)
  {
    return AT_ERROR;
  }

  switch (param[0])
  {
    case 'A':
      if (loraInfo->ClassB == 1)
      {
        ClassBEnableRequest = false;
      }
      errorStatus = LmHandlerRequestClass(CLASS_A);
      break;
    case 'B':
      if (loraInfo->ClassB == 1)
      {
        ClassBEnableRequest = true;
        errorStatus = LmHandlerRequestClass(CLASS_B);  /*Class B AT cmd switch Class B not supported cf.[UM2073]*/
      }
      else
      {
        return AT_NO_CLASS_B_ENABLE;
      }
      break;
    case 'C':
      errorStatus = LmHandlerRequestClass(CLASS_C);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  if (errorStatus == LORAMAC_HANDLER_NO_NETWORK_JOINED){
    return AT_NO_NET_JOINED;
  }
  else if (errorStatus == LORAMAC_HANDLER_SUCCESS){
	  nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	  AT_PPRINTF("+CLASS: %c\r\n", param[0]);
	  return AT_OK;
  }
  else{
    return AT_ERROR;
  }
  /* USER CODE BEGIN AT_DeviceClass_set_2 */

  /* USER CODE END AT_DeviceClass_set_2 */
}

ATEerror_t AT_DutyCycle_get(const char *param)
{
  /* USER CODE BEGIN AT_DutyCycle_get_1 */

  /* USER CODE END AT_DutyCycle_get_1 */
  bool dutyCycleEnable;
  if (LmHandlerGetDutyCycleEnable(&dutyCycleEnable) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(dutyCycleEnable);
  return AT_OK;
  /* USER CODE BEGIN AT_DutyCycle_get_2 */

  /* USER CODE END AT_DutyCycle_get_2 */
}

ATEerror_t AT_DutyCycle_set(const char *param)
{
  /* USER CODE BEGIN AT_DutyCycle_set_1 */

  /* USER CODE END AT_DutyCycle_set_1 */
  switch (param[0])
  {
    case '0':
    case '1':
      LmHandlerSetDutyCycleEnable(param[0] - '0');
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_DutyCycle_set_2 */

  /* USER CODE END AT_DutyCycle_set_2 */
}

ATEerror_t AT_JoinAcceptDelay1_get(const char *param)
{
  /* USER CODE BEGIN AT_JoinAcceptDelay1_get_1 */

  /* USER CODE END AT_JoinAcceptDelay1_get_1 */
  uint32_t rxDelay;
  if (LmHandlerGetJoinRx1Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_u(rxDelay);
  return AT_OK;
  /* USER CODE BEGIN AT_JoinAcceptDelay1_get_2 */

  /* USER CODE END AT_JoinAcceptDelay1_get_2 */
}

ATEerror_t AT_JoinAcceptDelay1_set(const char *param)
{
  /* USER CODE BEGIN AT_JoinAcceptDelay1_set_1 */

  /* USER CODE END AT_JoinAcceptDelay1_set_1 */
  uint32_t rxDelay;
  if (tiny_sscanf(param, "%lu", &rxDelay) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetJoinRx1Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_JoinAcceptDelay1_set_2 */

  /* USER CODE END AT_JoinAcceptDelay1_set_2 */
}

ATEerror_t AT_JoinAcceptDelay2_get(const char *param)
{
  /* USER CODE BEGIN AT_JoinAcceptDelay2_get_1 */

  /* USER CODE END AT_JoinAcceptDelay2_get_1 */
  uint32_t rxDelay;
  if (LmHandlerGetJoinRx2Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_u(rxDelay);
  return AT_OK;
  /* USER CODE BEGIN AT_JoinAcceptDelay2_get_2 */

  /* USER CODE END AT_JoinAcceptDelay2_get_2 */
}

ATEerror_t AT_JoinAcceptDelay2_set(const char *param)
{
  /* USER CODE BEGIN AT_JoinAcceptDelay2_set_1 */

  /* USER CODE END AT_JoinAcceptDelay2_set_1 */
  uint32_t rxDelay;
  if (tiny_sscanf(param, "%lu", &rxDelay) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetJoinRx2Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_JoinAcceptDelay2_set_2 */

  /* USER CODE END AT_JoinAcceptDelay2_set_2 */
}

ATEerror_t AT_Rx1Delay_get(const char *param)
{
  /* USER CODE BEGIN AT_Rx1Delay_get_1 */

  /* USER CODE END AT_Rx1Delay_get_1 */
  uint32_t rxDelay;
  if (LmHandlerGetRx1Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_u(rxDelay);
  return AT_OK;
  /* USER CODE BEGIN AT_Rx1Delay_get_2 */

  /* USER CODE END AT_Rx1Delay_get_2 */
}

ATEerror_t AT_Rx1Delay_set(const char *param)
{
  /* USER CODE BEGIN AT_Rx1Delay_set_1 */

  /* USER CODE END AT_Rx1Delay_set_1 */
  uint32_t rxDelay;
  if (tiny_sscanf(param, "%lu", &rxDelay) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetRx1Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Rx1Delay_set_2 */

  /* USER CODE END AT_Rx1Delay_set_2 */
}

ATEerror_t AT_Rx2Delay_get(const char *param)
{
  /* USER CODE BEGIN AT_Rx2Delay_get_1 */

  /* USER CODE END AT_Rx2Delay_get_1 */
  uint32_t rxDelay;
  if (LmHandlerGetRx2Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_u(rxDelay);
  return AT_OK;
  /* USER CODE BEGIN AT_Rx2Delay_get_2 */

  /* USER CODE END AT_Rx2Delay_get_2 */
}

ATEerror_t AT_Rx2Delay_set(const char *param)
{
  /* USER CODE BEGIN AT_Rx2Delay_set_1 */

  /* USER CODE END AT_Rx2Delay_set_1 */
  uint32_t rxDelay;
  if (tiny_sscanf(param, "%lu", &rxDelay) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetRx2Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Rx2Delay_set_2 */

  /* USER CODE END AT_Rx2Delay_set_2 */
}

ATEerror_t AT_Rx2DataRate_get(const char *param)
{
  /* USER CODE BEGIN AT_Rx2DataRate_get_1 */

  /* USER CODE END AT_Rx2DataRate_get_1 */
  RxChannelParams_t rx2Params;
  LmHandlerGetRX2Params(&rx2Params);
  print_d(rx2Params.Datarate);
  return AT_OK;
  /* USER CODE BEGIN AT_Rx2DataRate_get_2 */

  /* USER CODE END AT_Rx2DataRate_get_2 */
}

ATEerror_t AT_Rx2DataRate_set(const char *param)
{
  /* USER CODE BEGIN AT_Rx2DataRate_set_1 */

  /* USER CODE END AT_Rx2DataRate_set_1 */
  RxChannelParams_t rx2Params;

  /* Get the current configuration of RX2 */
  LmHandlerGetRX2Params(&rx2Params);

  /* Update the Datarate with scanf */
  if (tiny_sscanf(param, "%hhu", &(rx2Params.Datarate)) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (rx2Params.Datarate > 15)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetRX2Params(&rx2Params) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Rx2DataRate_set_2 */

  /* USER CODE END AT_Rx2DataRate_set_2 */
}

ATEerror_t AT_Rx2Frequency_get(const char *param)
{
  /* USER CODE BEGIN AT_Rx2Frequency_get_1 */

  /* USER CODE END AT_Rx2Frequency_get_1 */
  RxChannelParams_t rx2Params;
  LmHandlerGetRX2Params(&rx2Params);
  print_d(rx2Params.Frequency);
  return AT_OK;
  /* USER CODE BEGIN AT_Rx2Frequency_get_2 */

  /* USER CODE END AT_Rx2Frequency_get_2 */
}

ATEerror_t AT_Rx2Frequency_set(const char *param)
{
  /* USER CODE BEGIN AT_Rx2Frequency_set_1 */

  /* USER CODE END AT_Rx2Frequency_set_1 */
  RxChannelParams_t rx2Params;

  /* Get the current configuration of RX2 */
  LmHandlerGetRX2Params(&rx2Params);

  /* Update the frequency with scanf */
  if (tiny_sscanf(param, "%lu", &(rx2Params.Frequency)) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetRX2Params(&rx2Params) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_Rx2Frequency_set_2 */

  /* USER CODE END AT_Rx2Frequency_set_2 */
}

ATEerror_t AT_TransmitPower_get(const char *param)
{
  /* USER CODE BEGIN AT_TransmitPower_get_1 */

  /* USER CODE END AT_TransmitPower_get_1 */
  int8_t txPower;
  if (LmHandlerGetTxPower(&txPower) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(txPower);
  return AT_OK;
  /* USER CODE BEGIN AT_TransmitPower_get_2 */

  /* USER CODE END AT_TransmitPower_get_2 */
}

ATEerror_t AT_TransmitPower_set(const char *param)
{
  /* USER CODE BEGIN AT_TransmitPower_set_1 */

  /* USER CODE END AT_TransmitPower_set_1 */
  int8_t txPower;
  if (tiny_sscanf(param, "%hhu", &txPower) != 1)
  {
    return AT_PARAM_ERROR;
  }

  if (LmHandlerSetTxPower(txPower) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_TransmitPower_set_2 */

  /* USER CODE END AT_TransmitPower_set_2 */
}

ATEerror_t AT_PingSlot_get(const char *param)
{
  /* USER CODE BEGIN AT_PingSlot_get_1 */

  /* USER CODE END AT_PingSlot_get_1 */
  uint8_t periodicity;

  if (LmHandlerGetPingPeriodicity(&periodicity) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  print_d(periodicity);
  return AT_OK;
  /* USER CODE BEGIN AT_PingSlot_get_2 */

  /* USER CODE END AT_PingSlot_get_2 */
}

ATEerror_t AT_PingSlot_set(const char *param)
{
  /* USER CODE BEGIN AT_PingSlot_set_1 */

  /* USER CODE END AT_PingSlot_set_1 */
  uint8_t periodicity;

  if (tiny_sscanf(param, "%hhu", &periodicity) != 1)
  {
    return AT_PARAM_ERROR;
  }
  else if (periodicity > 7)
  {
    return AT_PARAM_ERROR;
  }
  else if (LmHandlerSetPingPeriodicity(periodicity) != LORAMAC_HANDLER_SUCCESS)
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
  /* USER CODE BEGIN AT_PingSlot_set_2 */

  /* USER CODE END AT_PingSlot_set_2 */
}

/* --------------- Radio tests commands --------------- */
ATEerror_t AT_test_txTone(const char *param)
{
  /* USER CODE BEGIN AT_test_txTone_1 */

  /* USER CODE END AT_test_txTone_1 */
  if (0U == TST_TxTone())
  {
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
  /* USER CODE BEGIN AT_test_txTone_2 */

  /* USER CODE END AT_test_txTone_2 */
}

ATEerror_t AT_test_rxRssi(const char *param)
{
  /* USER CODE BEGIN AT_test_rxRssi_1 */

  /* USER CODE END AT_test_rxRssi_1 */
  if (0U == TST_RxRssi())
  {
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
  /* USER CODE BEGIN AT_test_rxRssi_2 */

  /* USER CODE END AT_test_rxRssi_2 */
}

ATEerror_t AT_test_get_config(const char *param)
{
  /* USER CODE BEGIN AT_test_get_config_1 */

  /* USER CODE END AT_test_get_config_1 */
  testParameter_t testParam;
  uint32_t loraBW[7] = {7812, 15625, 31250, 62500, 125000, 250000, 500000};

  TST_get_config(&testParam);

  AT_PRINTF("1: Freq= %d Hz\r\n", testParam.freq);
  AT_PRINTF("2: Power= %d dBm\r\n", testParam.power);

  if ((testParam.modulation == TEST_FSK) || (testParam.modulation == TEST_MSK))
  {
    /*fsk*/
    AT_PRINTF("3: Bandwidth= %d Hz\r\n", testParam.bandwidth);
    AT_PRINTF("4: FSK datarate= %d bps\r\n", testParam.loraSf_datarate);
    AT_PRINTF("5: Coding Rate not applicable\r\n");
    AT_PRINTF("6: LNA State= %d  \r\n", testParam.lna);
    AT_PRINTF("7: PA Boost State= %d  \r\n", testParam.paBoost);
    if (testParam.modulation == TEST_FSK)
    {
      AT_PRINTF("8: modulation FSK\r\n");
    }
    else
    {
      AT_PRINTF("8: modulation MSK\r\n");
    }
    AT_PRINTF("9: Payload len= %d Bytes\r\n", testParam.payloadLen);
    if (testParam.modulation == TEST_FSK)
    {
      AT_PRINTF("10: FSK deviation= %d Hz\r\n", testParam.fskDev);
    }
    else
    {
      AT_PRINTF("10: FSK deviation forced to FSK datarate/4\r\n");
    }
    AT_PRINTF("11: LowDRopt not applicable\r\n");
    AT_PRINTF("12: FSK gaussian BT product= %d \r\n", testParam.BTproduct);
  }
  else if (testParam.modulation == TEST_LORA)
  {
    /*Lora*/
    AT_PRINTF("3: Bandwidth= %d (=%d Hz)\r\n", testParam.bandwidth, loraBW[testParam.bandwidth]);
    AT_PRINTF("4: SF= %d \r\n", testParam.loraSf_datarate);
    AT_PRINTF("5: CR= %d (=4/%d) \r\n", testParam.codingRate, testParam.codingRate + 4);
    AT_PRINTF("6: LNA State= %d  \r\n", testParam.lna);
    AT_PRINTF("7: PA Boost State= %d  \r\n", testParam.paBoost);
    AT_PRINTF("8: modulation LORA\r\n");
    AT_PRINTF("9: Payload len= %d Bytes\r\n", testParam.payloadLen);
    AT_PRINTF("10: Frequency deviation not applicable\r\n");
    AT_PRINTF("11: LowDRopt[0 to 2]= %d \r\n", testParam.lowDrOpt);
    AT_PRINTF("12 BT product not applicable\r\n");
  }
  else
  {
    AT_PRINTF("4: BPSK datarate= %d bps\r\n", testParam.loraSf_datarate);
  }
  return AT_OK;
  /* USER CODE BEGIN AT_test_get_config_2 */

  /* USER CODE END AT_test_get_config_2 */
}

ATEerror_t AT_test_set_config(const char *param)
{
  /* USER CODE BEGIN AT_test_set_config_1 */

  /* USER CODE END AT_test_set_config_1 */
  testParameter_t testParam = {0};
  uint32_t freq;
  int32_t power;
  uint32_t bandwidth;
  uint32_t loraSf_datarate;
  uint32_t codingRate;
  uint32_t lna;
  uint32_t paBoost;
  uint32_t modulation;
  uint32_t payloadLen;
  uint32_t fskDeviation;
  uint32_t lowDrOpt;
  uint32_t BTproduct;
  uint32_t crNum;

  if (13 == tiny_sscanf(param, "%d:%d:%d:%d:%d/%d:%d:%d:%d:%d:%d:%d:%d",
                        &freq,
                        &power,
                        &bandwidth,
                        &loraSf_datarate,
                        &crNum,
                        &codingRate,
                        &lna,
                        &paBoost,
                        &modulation,
                        &payloadLen,
                        &fskDeviation,
                        &lowDrOpt,
                        &BTproduct))
  {
    /*extend to new format for extended*/
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  /*get current config*/
  TST_get_config(&testParam);

  /* 8: modulation check and set */
  /* first check because required for others */
  if (modulation == TEST_FSK)
  {
    testParam.modulation = TEST_FSK;
  }
  else if (modulation == TEST_LORA)
  {
    testParam.modulation = TEST_LORA;
  }
  else if (modulation == TEST_BPSK)
  {
    testParam.modulation = TEST_BPSK;
  }
  else if (modulation == TEST_MSK)
  {
    testParam.modulation = TEST_MSK;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 1: frequency check and set */
  if (freq < 1000)
  {
    /*given in MHz*/
    testParam.freq = freq * 1000000;
  }
  else
  {
    testParam.freq = freq;
  }

  /* 2: power check and set */
  if ((power >= -9) && (power <= 22))
  {
    testParam.power = power;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 3: bandwidth check and set */
  if ((testParam.modulation == TEST_FSK) && (bandwidth >= 4800) && (bandwidth <= 467000))
  {
    testParam.bandwidth = bandwidth;
  }
  else if ((testParam.modulation == TEST_MSK) && (bandwidth >= 4800) && (bandwidth <= 467000))
  {
    testParam.bandwidth = bandwidth;
  }
  else if ((testParam.modulation == TEST_LORA) && (bandwidth <= BW_500kHz))
  {
    testParam.bandwidth = bandwidth;
  }
  else if (testParam.modulation == TEST_BPSK)
  {
    /* Not used */
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 4: datarate/spreading factor check and set */
  if ((testParam.modulation == TEST_FSK) && (loraSf_datarate >= 600) && (loraSf_datarate <= 300000))
  {
    testParam.loraSf_datarate = loraSf_datarate;
  }
  else if ((testParam.modulation == TEST_MSK) && (loraSf_datarate >= 100) && (loraSf_datarate <= 300000))
  {
    testParam.loraSf_datarate = loraSf_datarate;
  }
  else if ((testParam.modulation == TEST_LORA) && (loraSf_datarate >= 5) && (loraSf_datarate <= 12))
  {
    testParam.loraSf_datarate = loraSf_datarate;
  }
  else if ((testParam.modulation == TEST_BPSK) && (loraSf_datarate <= 1000))
  {
    testParam.loraSf_datarate = loraSf_datarate;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 5: coding rate check and set */
  if ((testParam.modulation == TEST_FSK) || (testParam.modulation == TEST_MSK) || (testParam.modulation == TEST_BPSK))
  {
    /* Not used */
  }
  else if ((testParam.modulation == TEST_LORA) && ((codingRate >= 5) && (codingRate <= 8)))
  {
    testParam.codingRate = codingRate - 4;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 6: lna state check and set */
  if (lna <= 1)
  {
    testParam.lna = lna;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 7: pa boost check and set */
  if (paBoost <= 1)
  {
    /* Not used */
    testParam.paBoost = paBoost;
  }

  /* 9: payloadLen check and set */
  if ((payloadLen != 0) && (payloadLen < 256))
  {
    testParam.payloadLen = payloadLen;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 10: fsk Deviation check and set */
  if ((testParam.modulation == TEST_LORA) || (testParam.modulation == TEST_BPSK) || (testParam.modulation == TEST_MSK))
  {
    /* Not used */
  }
  else if ((testParam.modulation == TEST_FSK) && ((fskDeviation >= 600) && (fskDeviation <= 200000)))
  {
    /*given in MHz*/
    testParam.fskDev = fskDeviation;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 11: low datarate optimization check and set */
  if ((testParam.modulation == TEST_FSK) || (testParam.modulation == TEST_BPSK) || (testParam.modulation == TEST_MSK))
  {
    /* Not used */
  }
  else if ((testParam.modulation == TEST_LORA) && (lowDrOpt <= 2))
  {
    testParam.lowDrOpt = lowDrOpt;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  /* 12: FSK gaussian BT product check and set */
  if ((testParam.modulation == TEST_LORA) || (testParam.modulation == TEST_BPSK))
  {
    /* Not used */
  }
  else if (((testParam.modulation == TEST_FSK) || (testParam.modulation == TEST_MSK)) && (BTproduct <= 4))
  {
    /*given in MHz*/
    testParam.BTproduct = BTproduct;
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  TST_set_config(&testParam);

  return AT_OK;
  /* USER CODE BEGIN AT_test_set_config_2 */

  /* USER CODE END AT_test_set_config_2 */
}

ATEerror_t AT_test_tx(const char *param)
{
  /* USER CODE BEGIN AT_test_tx_1 */

  /* USER CODE END AT_test_tx_1 */
  const char *buf = param;
  uint32_t nb_packet;

  if (1 != tiny_sscanf(buf, "%u", &nb_packet))
  {
    AT_PRINTF("AT+TTX: nb packets sent is missing\r\n");
    return AT_PARAM_ERROR;
  }

  if (0U == TST_TX_Start(nb_packet))
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
  /* USER CODE BEGIN AT_test_tx_2 */

  /* USER CODE END AT_test_tx_2 */
}

ATEerror_t AT_test_rx(const char *param)
{
  /* USER CODE BEGIN AT_test_rx_1 */

  /* USER CODE END AT_test_rx_1 */
  const char *buf = param;
  uint32_t nb_packet;

  if (1 != tiny_sscanf(buf, "%u", &nb_packet))
  {
    AT_PRINTF("AT+TRX: nb expected packets is missing\r\n");
    return AT_PARAM_ERROR;
  }

  if (0U == TST_RX_Start(nb_packet))
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
  /* USER CODE BEGIN AT_test_rx_2 */

  /* USER CODE END AT_test_rx_2 */
}

ATEerror_t AT_test_tx_hopping(const char *param)
{
  /* USER CODE BEGIN AT_test_tx_hopping_1 */

  /* USER CODE END AT_test_tx_hopping_1 */
  const char *buf = param;
  uint32_t freq_start;
  uint32_t freq_stop;
  uint32_t delta_f;
  uint32_t nb_tx;

  testParameter_t test_param;
  uint32_t hop_freq;

  if (4 != tiny_sscanf(buf, "%u,%u,%u,%u", &freq_start, &freq_stop, &delta_f, &nb_tx))
  {
    return AT_PARAM_ERROR;
  }

  /*if freq is set in MHz, convert to Hz*/
  if (freq_start < 1000)
  {
    freq_start *= 1000000;
  }
  if (freq_stop < 1000)
  {
    freq_stop *= 1000000;
  }
  /**/
  hop_freq = freq_start;

  for (int32_t i = 0; i < nb_tx; i++)
  {
    /*get current config*/
    TST_get_config(&test_param);

    /*increment frequency*/
    test_param.freq = hop_freq;
    /*Set new config*/
    TST_set_config(&test_param);

    APP_TPRINTF("Tx Hop at %dHz. %d of %d\r\n", hop_freq, i, nb_tx);

    if (0U != TST_TX_Start(1))
    {
      return AT_BUSY_ERROR;
    }

    hop_freq += delta_f;

    if (hop_freq > freq_stop)
    {
      hop_freq = freq_start;
    }
  }

  return AT_OK;
  /* USER CODE BEGIN AT_test_tx_hopping_2 */

  /* USER CODE END AT_test_tx_hopping_2 */
}

ATEerror_t AT_test_stop(const char *param)
{
  /* USER CODE BEGIN AT_test_stop_1 */

  /* USER CODE END AT_test_stop_1 */
  TST_stop();
  AT_PRINTF("Test Stop\r\n");
  return AT_OK;
  /* USER CODE BEGIN AT_test_stop_2 */

  /* USER CODE END AT_test_stop_2 */
}

/* --------------- Radio access commands --------------- */
ATEerror_t AT_write_register(const char *param)
{
  /* USER CODE BEGIN AT_write_register_1 */

  /* USER CODE END AT_write_register_1 */
  uint8_t add[2];
  uint16_t add16;
  uint8_t data;

  if (strlen(param) != 7)
  {
    return AT_PARAM_ERROR;
  }

  if (stringToData(param, add, 2) != 0)
  {
    return AT_PARAM_ERROR;
  }
  param += 5;
  if (stringToData(param, &data, 1) != 0)
  {
    return AT_PARAM_ERROR;
  }
  add16 = (((uint16_t)add[0]) << 8) + (uint16_t)add[1];
  Radio.Write(add16, data);

  return AT_OK;
  /* USER CODE BEGIN AT_write_register_2 */

  /* USER CODE END AT_write_register_2 */
}

ATEerror_t AT_read_register(const char *param)
{
  /* USER CODE BEGIN AT_read_register_1 */

  /* USER CODE END AT_read_register_1 */
  uint8_t add[2];
  uint16_t add16;
  uint8_t data;

  if (strlen(param) != 4)
  {
    return AT_PARAM_ERROR;
  }

  if (stringToData(param, add, 2) != 0)
  {
    return AT_PARAM_ERROR;
  }

  add16 = (((uint16_t)add[0]) << 8) + (uint16_t)add[1];
  data = Radio.Read(add16);
  AT_PRINTF("REG 0x%04X=0x%02X", add16, data);

  return AT_OK;
  /* USER CODE BEGIN AT_read_register_2 */

  /* USER CODE END AT_read_register_2 */
}

/* --------------- LoraWAN Certif command --------------- */
ATEerror_t AT_Certif(const char *param)
{
  /* USER CODE BEGIN AT_Certif_1 */

  /* USER CODE END AT_Certif_1 */
  switch (param[0])
  {
    case '0':
      LmHandlerJoin(ACTIVATION_TYPE_ABP, true);
    case '1':
      LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  UTIL_TIMER_Create(&TxCertifTimer, 8000, UTIL_TIMER_ONESHOT, OnCertifTimer, NULL);  /* 8s */
  UTIL_TIMER_Start(&TxCertifTimer);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaCertifTx), UTIL_SEQ_RFU, CertifSend);

  return AT_OK;
  /* USER CODE BEGIN AT_Certif_2 */

  /* USER CODE END AT_Certif_2 */
}

/* --------------- Information command --------------- */
ATEerror_t AT_bat_get(const char *param)
{
  /* USER CODE BEGIN AT_bat_get_1 */

  /* USER CODE END AT_bat_get_1 */
  print_d(SYS_GetBatteryLevel());

  return AT_OK;
  /* USER CODE BEGIN AT_bat_get_2 */

  /* USER CODE END AT_bat_get_2 */
}

/* USER CODE BEGIN EF */
// INSET commands
ATEerror_t AT_wdt_set(const char *param){
	uint8_t wdt;
	if (strcasecmp(param, "ON") == 0){
		wdt = 1U;
	}
	else if (strcasecmp(param, "OFF") == 0){
		wdt = 0U;
	}
	else{
		return AT_PARAM_ERROR;
	}

	uint8_t changed = (eeprom_vars.wdt != wdt);

	if (eeprom_set_wdt(wdt) != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+WDT: %s\r\n", wdt ? "ON" : "OFF");

	if (changed){
		(void)AT_nvm_store(0U);     /* changed configuration must not be lost by the reset */
		AT_flush_output();
		NVIC_SystemReset();
		}
	return AT_OK;
}

ATEerror_t AT_wdt_get(const char *param){
	uint8_t wdt = eeprom_vars.wdt;
	AT_PPRINTF("+WDT: %s\r\n", wdt ? "ON" : "OFF");
	return AT_OK;
}

ATEerror_t AT_log_set(const char *param){
	if (strcasecmp(param, "QUIET") == 0){
		UTIL_ADV_TRACE_SetVerboseLevel(VLEVEL_OFF);
		ram_vars.log_level = 0;
	}
	else if (strcasecmp(param, "DEBUG") == 0){
		UTIL_ADV_TRACE_SetVerboseLevel(VLEVEL_H);
		ram_vars.log_level = 1;
	}
	else{
		return AT_PARAM_ERROR;
	}
	AT_PPRINTF("+LOG: %s\r\n", ram_vars.log_level ? "DEBUG" : "QUIET");
	APP_LOG(TS_OFF, VLEVEL_H, "LOG is debug\r\n");
	return AT_OK;
}

/* Sub-GHz radio of the STM32WL: 150..960 MHz (RM0461). Radio.CheckRfFrequency() always returns true
   (radio.c), so the range is checked here */
#define AT_RF_FREQ_MIN_HZ   150000000UL
#define AT_RF_FREQ_MAX_HZ   960000000UL

static bool AT_rf_freq_valid(uint32_t freq){
	return (freq >= AT_RF_FREQ_MIN_HZ) && (freq <= AT_RF_FREQ_MAX_HZ);
}

/* EU868 band of a frequency: same table as VerifyRfFreq() in RegionEU868.c (static there) */
static bool AT_ch_get_band(uint32_t freq, uint8_t *band){
	if (!AT_rf_freq_valid(freq)){
		return false;
	}
	if ((freq >= 863000000UL) && (freq < 865000000UL)){
		*band = 2;
	}
	else if ((freq >= 865000000UL) && (freq <= 868000000UL)){
		*band = 0;
	}
	else if ((freq > 868000000UL) && (freq <= 868600000UL)){
		*band = 1;
	}
	else if ((freq >= 868700000UL) && (freq <= 869200000UL)){
		*band = 5;
	}
	else if ((freq >= 869400000UL) && (freq <= 869650000UL)){
		*band = 3;
	}
	else if ((freq >= 869700000UL) && (freq <= 870000000UL)){
		*band = 4;
	}
	else{
		return false;
	}
	return true;
}

/* "868.1" -> 868100000 Hz, up to 6 decimals, no float */
static bool AT_ch_parse_mhz(const char **p, uint32_t *hz){
	char *end;
	uint32_t mhz = strtoul(*p, &end, 10);
	uint32_t frac = 0;
	uint32_t mul = 100000UL;

	/* > 960 MHz: out of the radio range, also avoids the uint32 overflow of mhz * 1000000 */
	if ((end == *p) || (mhz > (AT_RF_FREQ_MAX_HZ / 1000000UL))){
		return false;
	}
	if (*end == '.'){
		end++;
		while ((*end >= '0') && (*end <= '9')){
			if (mul == 0){
				return false;
			}
			frac += (uint32_t)(*end - '0') * mul;
			mul /= 10;
			end++;
		}
	}
	*hz = (mhz * 1000000UL) + frac;
	*p = end;
	return true;
}

/* AT+CH=<ch>,<MHz>,DR<min>,DR<max> -> +CH: <ch>,<Hz>,DR<min>:DR<max> */
ATEerror_t AT_ch_set(const char *param){
	const char *p = param;
	char *end;
	uint32_t id;
	uint32_t freq;
	uint32_t dr_min;
	uint32_t dr_max;
	MibRequestConfirm_t mib;
	LoRaMacNvmData_t *nvm;
	ChannelParams_t *ch;

	id = strtoul(p, &end, 10);
	if ((end == p) || (*end != ',') || (id >= 16U)){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	if (!AT_ch_parse_mhz(&p, &freq) || (*p != ',')){
		return AT_PARAM_ERROR;
	}
	p++;
	if (strncasecmp(p, "DR", 2) != 0){
		return AT_PARAM_ERROR;
	}
	dr_min = strtoul(p + 2, &end, 10);
	if ((end == p + 2) || (*end != ',') || (strncasecmp(end + 1, "DR", 2) != 0)){
		return AT_PARAM_ERROR;
	}
	p = end + 3;
	dr_max = strtoul(p, &end, 10);
	if ((end == p) || (*end != '\0') || (dr_min > dr_max) || (dr_max > 7U)){
		return AT_PARAM_ERROR;
	}

	mib.Type = MIB_NVM_CTXS;
	LoRaMacMibGetRequestConfirm(&mib);
	nvm = (LoRaMacNvmData_t *)mib.Param.Contexts;
	ch = &nvm->RegionGroup2.Channels[id];

	/* Same values already in NVM (and channel enabled): nothing to set, nothing to store */
	if ((ch->Frequency == freq) &&
		(ch->DrRange.Fields.Min == (int8_t)dr_min) &&
		(ch->DrRange.Fields.Max == (int8_t)dr_max) &&
		(((nvm->RegionGroup2.ChannelsMask[0] >> id) & 1U) != 0U)){
		AT_PPRINTF("+CH: %u,%u,DR%u:DR%u\r\n", id, ch->Frequency,
				(unsigned)ch->DrRange.Fields.Min, (unsigned)ch->DrRange.Fields.Max);
		return AT_OK;
	}

	/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
	if (LoRaMacIsBusy()){
		return AT_BUSY_ERROR;
	}

	uint8_t band;
	bool in_plan = AT_ch_get_band(freq, &band);

	if (!in_plan){
		/* Out of the EU868 sub-bands: allowed only with CT OFF (Seeed: customized band plans),
		   the radio must still support the frequency */
		if ((eeprom_vars.ct != 0U) || !AT_rf_freq_valid(freq)){
			return AT_PARAM_ERROR;
		}
		band = 0U;   /* duty cycle accounting as 865-868 MHz (1 %); DC is off with the ESP driver */
	}

	if ((id < 3U) || !in_plan){
		/* Default channels: LoRaMacChannelAdd() refuses them (the server must not change them);
		   out-of-plan frequencies: RegionEU868ChannelAdd() refuses them (VerifyRfFreq).
		   Write them directly into the NVM context. Rx1Frequency (AT+RXWIN1) is kept. */
		ch->Frequency = freq;
		ch->DrRange.Fields.Min = (int8_t)dr_min;
		ch->DrRange.Fields.Max = (int8_t)dr_max;
		ch->Band = band;
		nvm->RegionGroup2.ChannelsMask[0] |= (uint16_t)(1U << id);
	}
	else{
		ChannelParams_t params = *ch;    /* keeps Rx1Frequency (AT+RXWIN1) */
		params.Frequency = freq;
		params.DrRange.Fields.Min = (int8_t)dr_min;
		params.DrRange.Fields.Max = (int8_t)dr_max;
		if (LoRaMacChannelAdd((uint8_t)id, params) != LORAMAC_STATUS_OK){
			return AT_PARAM_ERROR;
		}
	}

	/* Channels live in the LoRaWAN NVM context: store it to flash
	   (LoRaMacHalt() inside detects the change through the RegionGroup2 CRC) */
//	if (LmHandlerNvmDataStore() == LORAMAC_HANDLER_ERROR){
//		return AT_ERROR;
//	}

	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+CH: %u,%u,DR%u:DR%u\r\n", id, ch->Frequency,
			(unsigned)ch->DrRange.Fields.Min, (unsigned)ch->DrRange.Fields.Max);
	return AT_OK;
}

void AT_ch_debug_print(uint8_t id){
	MibRequestConfirm_t mib;
	const LoRaMacNvmData_t *ram;
	const LoRaMacNvmData_t *flash = (const LoRaMacNvmData_t *)LORAWAN_NVM_BASE_ADDRESS;
	const ChannelParams_t *ch;

	if (id >= 16U){
		return;
	}

	mib.Type = MIB_NVM_CTXS;
	LoRaMacMibGetRequestConfirm(&mib);
	ram = (const LoRaMacNvmData_t *)mib.Param.Contexts;

	ch = &ram->RegionGroup2.Channels[id];
	AT_PPRINTF("CH%u RAM:   freq=%u rx1=%u DR%d:DR%d band=%u en=%u\r\n", id,
			ch->Frequency, ch->Rx1Frequency,
			ch->DrRange.Fields.Min, ch->DrRange.Fields.Max, ch->Band,
			(unsigned)((ram->RegionGroup2.ChannelsMask[0] >> id) & 1U));

	ch = &flash->RegionGroup2.Channels[id];
	AT_PPRINTF("CH%u FLASH: freq=%u rx1=%u DR%d:DR%d band=%u en=%u\r\n", id,
			ch->Frequency, ch->Rx1Frequency,
			ch->DrRange.Fields.Min, ch->DrRange.Fields.Max, ch->Band,
			(unsigned)((flash->RegionGroup2.ChannelsMask[0] >> id) & 1U));
}

ATEerror_t AT_ch_get(const char *param){
	// TODO THIS IS JUST DEBUG (not in the command table)
	AT_ch_debug_print(0);
	return AT_OK;
}

/* AT+MODE=LWABP|TEST -> +MODE: LWABP|TEST */
ATEerror_t AT_mode_set(const char *param){
	if (strcasecmp(param, "LWABP") == 0){
		if (ram_vars.mode == 1U){
			/* TEST -> LWABP: radio test functions replaced the LoRaMac radio callbacks */
			TST_stop();
			LoRaWAN_Reinit();
			ram_vars.mode = 0U;
		}
		LoRaWAN_StartAbp();
	}
	else if (strcasecmp(param, "TEST") == 0){
		if (ram_vars.mode == 0U){
			/* LWABP -> TEST: stop LoRaMac, LmHandlerHalt() also refreshes the RAM backup
			   of the NVM context used by LoRaWAN_Reinit() */
			if (LoRaMacIsBusy()){
				return AT_BUSY_ERROR;
			}
			LmHandlerHalt();
			LoRaMacStop();
			ram_vars.mode = 1U;
		}
	}
	else{
		return AT_PARAM_ERROR;
	}

	AT_PPRINTF("+MODE: %s\r\n", (ram_vars.mode == 1U) ? "TEST" : "LWABP");
	return AT_OK;
}

ATEerror_t AT_mode_get(const char *param){
	AT_PPRINTF("+MODE: %s\r\n", (ram_vars.mode == 1U) ? "TEST" : "LWABP");
	return AT_OK;
}

/* LoRaWAN TX power. The stack computes the radio power at every TX (RegionCommonComputeTxPower):
     radio dBm = floor(MaxEirp - 2 * index - AntennaGain)      (EU868: MaxEirp = 16)
   FORCE:    index 0, AntennaGain = MaxEirp - dBm  -> exactly <dBm> on the radio (gain is only a knob)
   no FORCE: AntennaGain = region default (2.15), <dBm> is the EIRP from the region table
             (16, 14, ..., 2), index = (MaxEirp - dBm) / 2 */
static LoRaMacNvmData_t *AT_power_nvm(void){
	MibRequestConfirm_t mib;
	mib.Type = MIB_NVM_CTXS;
	LoRaMacMibGetRequestConfirm(&mib);
	return (LoRaMacNvmData_t *)mib.Param.Contexts;
}

static float AT_power_default_gain(const LoRaMacNvmData_t *nvm){
	GetPhyParams_t get;
	get.Attribute = PHY_DEF_ANTENNA_GAIN;
	return RegionGetPhyParam(nvm->MacGroup2.Region, &get).fValue;
}

/* Prints the current setting: FORCE when the antenna gain differs from the region default */
static void AT_power_print(const LoRaMacNvmData_t *nvm){
	const LoRaMacParams_t *mp = &nvm->MacGroup2.MacParams;
	int8_t idx = nvm->MacGroup1.ChannelsTxPower;

	if (mp->AntennaGain != AT_power_default_gain(nvm)){
		AT_PPRINTF("+POWER: %d, FORCE\r\n",
				(int)floorf(mp->MaxEirp - (2.0f * idx) - mp->AntennaGain));
	}
	else{
		AT_PPRINTF("+POWER: %d\r\n", (int)mp->MaxEirp - (2 * idx));
	}
}

/* AT+POWER=<dBm>[,FORCE] -> +POWER: <dBm>[, FORCE] */
ATEerror_t AT_power_set(const char *param){
	char *end;
	long dbm;
	bool force = false;
	LoRaMacNvmData_t *nvm;
	int8_t idx;
	float gain;
	MibRequestConfirm_t mib;

	dbm = strtol(param, &end, 10);
	if (end == param){
		return AT_PARAM_ERROR;
	}
	if (*end == ','){
		end++;
		while (*end == ' '){
			end++;
		}
		if (strcasecmp(end, "FORCE") != 0){
			return AT_PARAM_ERROR;
		}
		force = true;
	}
	else if (*end != '\0'){
		return AT_PARAM_ERROR;
	}

	nvm = AT_power_nvm();

	if (force){
		if ((dbm < -9) || (dbm > 22)){       /* HP PA range */
			return AT_PARAM_ERROR;
		}
		idx = 0;
		gain = nvm->MacGroup2.MacParams.MaxEirp - (float)dbm;
	}
	else{
		long diff = (long)nvm->MacGroup2.MacParams.MaxEirp - dbm;

		/* Not a value of the region table; the upper index limit (EU868: 7 = 2 dBm)
		   is checked by LmHandlerSetTxPower() below */
		if ((diff < 0) || ((diff % 2) != 0) || (diff > 30)){
			return AT_PARAM_ERROR;
		}
		idx = (int8_t)(diff / 2);
		gain = AT_power_default_gain(nvm);
	}

	/* Set and store only when something changes */
	if ((nvm->MacGroup1.ChannelsTxPower != idx) ||
		(nvm->MacGroup2.ChannelsTxPowerDefault != idx) ||
		(nvm->MacGroup2.MacParams.AntennaGain != gain) ||
		(nvm->MacGroup2.MacParamsDefaults.AntennaGain != gain)){

		/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
		if (LoRaMacIsBusy()){
			return AT_BUSY_ERROR;
		}

		if (LmHandlerSetTxPower(idx) != LORAMAC_HANDLER_SUCCESS){    /* MIB + LmHandlerParams.TxPower */
			return AT_PARAM_ERROR;
		}
		mib.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
		mib.Param.ChannelsDefaultTxPower = idx;
		LoRaMacMibSetRequestConfirm(&mib);
		mib.Type = MIB_ANTENNA_GAIN;
		mib.Param.AntennaGain = gain;
		LoRaMacMibSetRequestConfirm(&mib);
		mib.Type = MIB_DEFAULT_ANTENNA_GAIN;
		mib.Param.DefaultAntennaGain = gain;
		LoRaMacMibSetRequestConfirm(&mib);

		/* Index and antenna gain live in the LoRaWAN NVM context: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
		nvm_dirty = 1U;
	}

	ram_vars.power_force = force ? 1U : 0U;
	AT_power_print(nvm);
	return AT_OK;
}

ATEerror_t AT_power_get(const char *param){
	AT_power_print(AT_power_nvm());
	return AT_OK;
}

/* AT+LW=DC,ON|OFF -> +LW: DC, ON|OFF, 0
   3rd field = MaxDCycle in Seeed (aggregated duty cycle 1/2^MaxDCycle, 0 = no limitation); not settable here */
static ATEerror_t AT_lw_dc_set(const char *val){
	uint8_t on;

	if (strcasecmp(val, "ON") == 0){
		on = 1U;
	}
	else if (strcasecmp(val, "OFF") == 0){
		on = 0U;
	}
	else{
		return AT_PARAM_ERROR;
	}
	/* Same function as AT_DutyCycle_set(): LoRaMacTestSetDutyCycleOn() -> Nvm.MacGroup2.DutyCycleOn.
	   EEPROM: LmHandlerConfigure() overwrites the stack value with the region default */
	LmHandlerSetDutyCycleEnable(on != 0U);
	if (eeprom_set_dc(on) != EE_OK){
		return AT_ERROR;
	}

	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+LW: DC, %s, 0\r\n", (on != 0U) ? "ON" : "OFF");
	return AT_OK;
}

/* LoRaWAN NVM context (channels, keys, DevAddr, FCnt, power, RX delays, NbTrans...) -> flash.
   Stored only when an AT command changed the configuration (nvm_dirty, set by the handlers) or force = 1.
   LmHandlerNvmDataStore() itself writes the flash only when the context CRC changed; every write erases
   a 2 KB page. FCntUp changes of the uplinks do not set nvm_dirty (not stored, by design for now).
   Not in TEST mode: NvmDataMgmtStoreEnd() restarts LoRaMac (LoRaMacStart); nvm_dirty stays set */
static ATEerror_t AT_nvm_store(uint8_t force){
	if ((nvm_dirty == 0U) && (force == 0U)){
		return AT_OK;
	}
	if (ram_vars.mode != 0U){
		return AT_ERROR;
	}
	/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
	if (LoRaMacIsBusy()){
		return AT_BUSY_ERROR;
	}
	if (LmHandlerNvmDataStore() == LORAMAC_HANDLER_ERROR){
		return AT_ERROR;
	}
	nvm_dirty = 0U;
	return AT_OK;
}

/* AT+NVM -> +NVM: OK   (own command, not Seeed: store the LoRaWAN NVM context to flash now) */
ATEerror_t AT_nvm_run(const char *param){
	ATEerror_t status = AT_nvm_store(1U);

	if (status == AT_OK){
		AT_PPRINTF("+NVM: OK\r\n");
	}
	return status;
}

/* AT+LW=NET,ON|OFF -> +LW: NET, ON|OFF   (ON = public network sync word, OFF = private)
   NET,OFF is the last command of the ESP driver configuration block: the NVM context is stored here */
static ATEerror_t AT_lw_net_set(const char *val){
	MibRequestConfirm_t mib;
	uint8_t on;
	ATEerror_t status;

	if (strcasecmp(val, "ON") == 0){
		on = 1U;
	}
	else if (strcasecmp(val, "OFF") == 0){
		on = 0U;
	}
	else{
		return AT_PARAM_ERROR;
	}
	/* Nvm.MacGroup2.PublicNetwork + Radio.SetPublicNetwork() (sync word).
	   EEPROM: LmHandlerConfigure() overwrites it with LORAWAN_PUBLIC_NETWORK */
	mib.Type = MIB_PUBLIC_NETWORK;
	mib.Param.EnablePublicNetwork = (on != 0U);
	LoRaMacMibSetRequestConfirm(&mib);
	if (eeprom_set_net(on) != EE_OK){
		return AT_ERROR;
	}
	nvm_dirty = 1U;
	/* end of the ESP configuration block: store the changed configuration (in TEST mode it stays dirty) */
	if ((on == 0U) && (ram_vars.mode == 0U)){
		status = AT_nvm_store(0U);
		if (status != AT_OK){
			return status;
		}
	}

	AT_PPRINTF("+LW: NET, %s\r\n", (on != 0U) ? "ON" : "OFF");
	return AT_OK;
}

/* AT+LW=NET -> +LW: NET, ON|OFF (value from the stack) */
static ATEerror_t AT_lw_net_get(void){
	MibRequestConfirm_t mib;

	mib.Type = MIB_PUBLIC_NETWORK;
	LoRaMacMibGetRequestConfirm(&mib);
	AT_PPRINTF("+LW: NET, %s\r\n", mib.Param.EnablePublicNetwork ? "ON" : "OFF");
	return AT_OK;
}

/* ULDL (Seeed): ul = counter of the NEXT uplink (factory default 1), dl = downlink counter (default 0).
   Stack: FCntList.FCntUp = counter of the LAST sent uplink (0 after reset, LoRaMacCrypto.c sets it
   when securing a frame), so ul = FCntUp + 1. FCntDown = last received downlink counter
   (LoRaWAN 1.0.x), FCNT_DOWN_INITIAL_VALUE until the first downlink. */
static FCntList_t *AT_lw_fcnt(LoRaMacNvmData_t **nvm){
	MibRequestConfirm_t mib;

	mib.Type = MIB_NVM_CTXS;
	LoRaMacMibGetRequestConfirm(&mib);
	*nvm = (LoRaMacNvmData_t *)mib.Param.Contexts;
	return &(*nvm)->Crypto.FCntList;
}

/* AT+LW=ULDL -> +LW: ULDL <ul>, <dl> */
static ATEerror_t AT_lw_uldl_get(void){
	LoRaMacNvmData_t *nvm;
	FCntList_t *fcnt = AT_lw_fcnt(&nvm);
	uint32_t dl = fcnt->FCntDown;

	/* tiny_vsnprintf_like: no %l */
	AT_PPRINTF("+LW: ULDL %u, %u\r\n", (unsigned)(fcnt->FCntUp + 1U),   /* next uplink */
			(unsigned)((dl == FCNT_DOWN_INITIAL_VALUE) ? 0U : dl));   /* no downlink yet */
	return AT_OK;
}

/* AT+LW=ULDL,<ul>,<dl> -> +LW: ULDL <ul>, <dl>   (Seeed spec 4.28.2; ul = next uplink counter) */
static ATEerror_t AT_lw_uldl_set(const char *val){
	LoRaMacNvmData_t *nvm;
	FCntList_t *fcnt = AT_lw_fcnt(&nvm);
	char *end;
	uint32_t ul;
	uint32_t dl;
	uint32_t dl_stack;
	ATEerror_t status;

	/* Not in TEST mode: the NVM store below restarts LoRaMac (AT_nvm_store) */
	if (ram_vars.mode != 0U){
		return AT_ERROR;
	}
	ul = strtoul(val, &end, 10);
	if ((end == val) || (*end != ',')){
		return AT_PARAM_ERROR;
	}
	val = end + 1;
	while (*val == ' '){
		val++;
	}
	dl = strtoul(val, &end, 10);
	if ((end == val) || (*end != '\0')){
		return AT_PARAM_ERROR;
	}

	/* The stack keeps the LAST uplink counter: next = ul -> FCntUp = ul - 1.
	   ul = 0 is not possible: LoRaMacCryptoSecureMessage() refuses a counter smaller than FCntUp
	   (0xFFFFFFFF would block every uplink), so the lowest next counter is 1 (FCntUp = 0). */
	if (ul == 0U){
		ul = 1U;
	}
	/* dl = 0 means "no downlink yet" (same as the GET): FCNT_DOWN_INITIAL_VALUE, otherwise a downlink
	   with FCnt 0 (after a counter reset on the network server) would be refused as a duplicate */
	dl_stack = (dl == 0U) ? FCNT_DOWN_INITIAL_VALUE : dl;

	/* Set and store only when something changes */
	if ((fcnt->FCntUp != (ul - 1U)) || (fcnt->FCntDown != dl_stack)){
		/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
		if (LoRaMacIsBusy()){
			return AT_BUSY_ERROR;
		}
		fcnt->FCntUp = ul - 1U;
		fcnt->FCntDown = dl_stack;
		nvm->Crypto.LastDownFCnt = dl_stack;   /* the stack keeps it together with FCntDown */

		/* Counters live in the LoRaWAN NVM context (Crypto group): store it to flash */
		nvm_dirty = 1U;
		status = AT_nvm_store(0U);
		if (status != AT_OK){
			return status;
		}
	}

	AT_PPRINTF("+LW: ULDL %u, %u\r\n", (unsigned)ul, (unsigned)dl);    /* tiny_vsnprintf_like: no %l */
	return AT_OK;
}

/* AT+LW=LDRO,ON|OFF|AUTO -> +LW: LDRO, ON|OFF|AUTO
   Only echo: the radio driver keeps its automatic LDRO (on for SF11/SF12 at 125 kHz, SF12 at 250 kHz,
   radio.c RadioSetTxConfig/RadioSetRxConfig), which LoRaWAN requires on DR0/DR1.
   Not stored (same as Seeed: "this configuration is not memorable"). */
static ATEerror_t AT_lw_ldro_set(const char *val){
	const char *ldro;

	if (strcasecmp(val, "ON") == 0){
		ldro = "ON";
	}
	else if (strcasecmp(val, "OFF") == 0){
		ldro = "OFF";
	}
	else if (strcasecmp(val, "AUTO") == 0){
		ldro = "AUTO";
	}
	else{
		return AT_PARAM_ERROR;
	}
	AT_PPRINTF("+LW: LDRO, %s\r\n", ldro);
	return AT_OK;
}

/* Overrides the weak function in RegionEU868.c: LBT RSSI threshold [dBm], 0 = LBT off */
int16_t RegionEU868LbtThreshold(void){
	return eeprom_vars.thld_dbm;
}

/* AT+LW=THLD,<dBm> -> +LW: THLD, <dBm>
   Listen before talk before every uplink: a channel is used only when its RSSI stays below <dBm>
   for 6 ms; when all enabled channels are busy the uplink fails (LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND).
   0 = LBT off. Stored in EEPROM (the ESP driver sends it in its persistent configuration block). */
static ATEerror_t AT_lw_thld_set(const char *val){
	char *end;
	long thld = strtol(val, &end, 10);

	/* RSSI threshold -128..0 dBm, 0 = LBT off */
	if ((end == val) || (*end != '\0') || (thld < -128L) || (thld > 0L)){
		return AT_PARAM_ERROR;
	}
	if (eeprom_set_thld((int16_t)thld) != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+LW: THLD, %d\r\n", (int)eeprom_vars.thld_dbm);
	return AT_OK;
}

/* AT+LW=DUMRX: RAM only, OFF after every reset (the ESP driver sends it at every start) */
static uint8_t lw_dumrx = 0U;

/* Overrides the weak function in LoRaMac.c: DUMRX ON = no RX windows after an unconfirmed uplink */
bool LoRaMacSkipRxWindows(void){
	return (lw_dumrx != 0U);
}

/* AT+LW=DUMRX,ON|OFF -> +LW: DUMRX, ON|OFF
   ON: unconfirmed uplinks end right after the TX ("+MSGHEX: Done" without waiting for RX1/RX2);
   no downlinks / MAC commands can be received for them. Confirmed uplinks are not affected. */
static ATEerror_t AT_lw_dumrx_set(const char *val){
	if (strcasecmp(val, "ON") == 0){
		lw_dumrx = 1U;
	}
	else if (strcasecmp(val, "OFF") == 0){
		lw_dumrx = 0U;
	}
	else{
		return AT_PARAM_ERROR;
	}
	AT_PPRINTF("+LW: DUMRX, %s\r\n", (lw_dumrx != 0U) ? "ON" : "OFF");
	return AT_OK;
}

/* Overrides the weak function in LoRaMac.c: CT OFF = any valid downlink in RX1/RX2
   terminates a confirmed uplink (Seeed relaxation 1) */
bool LoRaMacAckRelaxed(void){
	return (eeprom_vars.ct == 0U);
}

/* AT+LW=CT,ON|OFF -> +LW: CT, ON|OFF
   ON = strict LoRaWAN compliance, OFF (default) = Seeed relaxations:
   1) any valid downlink terminates a confirmed uplink (LoRaMacAckRelaxed)
   4) AT+CH accepts frequencies out of the EU868 sub-bands (AT_ch_set) */
static ATEerror_t AT_lw_ct_set(const char *val){
	uint8_t on;

	if (strcasecmp(val, "ON") == 0){
		on = 1U;
	}
	else if (strcasecmp(val, "OFF") == 0){
		on = 0U;
	}
	else{
		return AT_PARAM_ERROR;
	}
	if (eeprom_set_ct(on) != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+LW: CT, %s\r\n", (on != 0U) ? "ON" : "OFF");
	return AT_OK;
}

/* GET: AT+LW=NET | AT+LW=ULDL (Seeed syntax, routed here by AT_lw_set) or AT+LW?NET | AT+LW?ULDL */
ATEerror_t AT_lw_get(const char *param){
	if (strcasecmp(param, "NET") == 0){
		return AT_lw_net_get();
	}
	if (strcasecmp(param, "ULDL") == 0){
		return AT_lw_uldl_get();
	}
	return AT_PARAM_ERROR;
}

/* SET: AT+LW=<par>,<value>
   Seeed reads a value with AT+LW=<par> (no value): the parser routes '=' to the set handler,
   so a parameter without a value is passed to AT_lw_get() */
ATEerror_t AT_lw_set(const char *param){
	const char *val = strchr(param, ',');
	size_t len;

	if (val == NULL){
		return AT_lw_get(param);
	}
	len = (size_t)(val - param);
	val++;
	while (*val == ' '){
		val++;
	}

	if ((len == 2U) && (strncasecmp(param, "DC", 2) == 0)){
		return AT_lw_dc_set(val);
	}
	if ((len == 3U) && (strncasecmp(param, "NET", 3) == 0)){
		return AT_lw_net_set(val);
	}
	if ((len == 4U) && (strncasecmp(param, "ULDL", 4) == 0)){
		return AT_lw_uldl_set(val);
	}
	if ((len == 4U) && (strncasecmp(param, "LDRO", 4) == 0)){
		return AT_lw_ldro_set(val);
	}
	if ((len == 2U) && (strncasecmp(param, "CT", 2) == 0)){
		return AT_lw_ct_set(val);
	}
	if ((len == 5U) && (strncasecmp(param, "DUMRX", 5) == 0)){
		return AT_lw_dumrx_set(val);
	}
	if ((len == 4U) && (strncasecmp(param, "THLD", 4) == 0)){
		return AT_lw_thld_set(val);
	}
	return AT_PARAM_ERROR;
}

/* AT+PORT=<1..255> -> +PORT: <port>
   LoRaWAN FPort used by MSG/CMSG/MSGHEX/CMSGHEX (Seeed spec 4.11, default 8).
   Stored in EEPROM (the ESP driver sends it in its persistent configuration block). */
ATEerror_t AT_port_set(const char *param){
	char *end;
	unsigned long port = strtoul(param, &end, 10);

	if ((end == param) || (*end != '\0') || (port < 1UL) || (port > 255UL)){
		return AT_PARAM_ERROR;
	}
	if (eeprom_set_port((uint8_t)port) != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+PORT: %u\r\n", (unsigned)eeprom_vars.port);
	return AT_OK;
}

/* AT+PORT? | AT+PORT -> +PORT: <port> */
ATEerror_t AT_port_get(const char *param){
	AT_PPRINTF("+PORT: %u\r\n", (unsigned)eeprom_vars.port);
	return AT_OK;
}

/* AT+RETRY=<0..254> -> +RETRY: <n>   (Seeed 4.17: total transmissions of a confirmed uplink, <2 = 1)
   Only stored here; AT+CMSGHEX sets it temporarily as NbTrans (AT_msghex_send) */
ATEerror_t AT_retry_set(const char *param){
	char *end;
	unsigned long retry = strtoul(param, &end, 10);

	if ((end == param) || (*end != '\0') || (retry > 254UL)){
		return AT_PARAM_ERROR;
	}
	if (eeprom_set_retry((uint8_t)retry) != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+RETRY: %u\r\n", (unsigned)eeprom_vars.retry);
	return AT_OK;
}

/* AT+DELAY=RX1|RX2,<ms> -> +DELAY: <ms>   (ESP driver expects the value without the window name)
   Stack: MIB_RECEIVE_DELAY_1/2 -> Nvm.MacGroup2.MacParams.ReceiveDelay1/2 (LoRaWAN NVM context) */
ATEerror_t AT_delay_set(const char *param){
	const char *val = strchr(param, ',');
	char *end;
	uint32_t rxDelay;

	/* AT+DELAY=RX1 | AT+DELAY=RX2 (no value) reads the delay, same as AT+LW=<par> */
	if (val == NULL){
		return AT_delay_get(param);
	}
	val++;
	rxDelay = strtoul(val, &end, 10);
	if ((end == val) || (*end != '\0')){
		return AT_PARAM_ERROR;
	}

	if (strncasecmp(param, "RX1,", 4) == 0){
		if (LmHandlerSetRx1Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS){
			return AT_PARAM_ERROR;
		}
	}
	else if (strncasecmp(param, "RX2,", 4) == 0){
		if (LmHandlerSetRx2Delay(rxDelay) != LORAMAC_HANDLER_SUCCESS){
			return AT_PARAM_ERROR;
		}
	}
	else{
		return AT_PARAM_ERROR;
	}

	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+DELAY: %u\r\n", (unsigned long)rxDelay);
	return AT_OK;
}

/* AT+DELAY?RX1|RX2 or AT+DELAY=RX1|RX2 -> +DELAY: <ms> */
ATEerror_t AT_delay_get(const char *param){
	uint32_t rxDelay;

	if (strcasecmp(param, "RX1") == 0){
		if (LmHandlerGetRx1Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS){
			return AT_ERROR;
		}
	}
	else if (strcasecmp(param, "RX2") == 0){
		if (LmHandlerGetRx2Delay(&rxDelay) != LORAMAC_HANDLER_SUCCESS){
			return AT_ERROR;
		}
	}
	else{
		return AT_PARAM_ERROR;
	}

	AT_PPRINTF("+DELAY: %u\r\n", (unsigned)rxDelay);    /* tiny_vsnprintf_like: no %l */
	return AT_OK;
}

/* AT+RXWIN1=0,0 -> +RXWIN1: 0,0
   Only what the ESP driver sends: RX1 on the uplink frequency (Channels[].Rx1Frequency = 0, the stack
   default after a reset; the server may change it with DlChannelReq). Other values are not supported. */
ATEerror_t AT_rxwin1_set(const char *param){
	const char *val = param;
	char *end;
	uint32_t id;
	uint32_t freq;

	id = strtoul(val, &end, 10);
	if ((end == val) || (*end != ',') || (id >= 16U)){
		return AT_PARAM_ERROR;
	}
	val = end + 1;
	freq = strtoul(val, &end, 10);
	if ((end == val) || (*end != '\0')){
		return AT_PARAM_ERROR;
	}
	if (id == 0 && freq == 0){
		AT_PPRINTF("+RXWIN1: 0,0\r\n");
		return AT_OK;
	}
	else{
		return AT_ERROR;
	}
}

/* "2B7E1516..." -> {0x2B, 0x7E, 0x15, 0x16, ...}, MSB first, at most max bytes.
   Optional quotes, ':' or ' ' between bytes accepted ("01:23:45:67", Seeed accepts both forms).
   Returns the number of bytes, -1 = format error or more than max bytes */
static int16_t AT_parse_hex(const char *p, uint8_t *out, uint16_t max){
	char byte[3] = {0};
	char *end;
	uint16_t n = 0;

	if (*p == '"'){
		p++;
	}
	for (;;){
		while ((*p == ':') || (*p == ' ')){
			p++;
		}
		if ((*p == '\0') || (*p == '"')){
			break;
		}
		if (n >= max){
			return -1;
		}
		/* two hex digits; strtoul alone would also take a sign ("+1", "-1") */
		if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1])){
			return -1;
		}
		byte[0] = p[0];
		byte[1] = p[1];
		out[n] = (uint8_t)strtoul(byte, &end, 16);
		n++;
		p += 2;
	}
	if (*p == '"'){
		p++;
	}
	return (*p == '\0') ? (int16_t)n : -1;
}

/* AT+KEY=NWKSKEY|APPSKEY, "<32 hex>" -> +KEY: NWKSKEY|APPSKEY <32 hex>
   MIB_NWK_S_KEY / MIB_APP_S_KEY (LoRaMacCryptoSetKey) work also when activated;
   LmHandlerSetKey() refuses them after the ABP activation. LoRaWAN 1.0.x uses NWK_S_KEY
   for the MIC and the FPort 0 payload (LoRaMacCrypto.c). Keys live in the NVM context (SecureElement). */
ATEerror_t AT_key_set(const char *param){
	const char *val = strchr(param, ',');
	size_t len;
	const char *name;
	uint8_t key[16];
	char hex[33];
	uint8_t i;
	MibRequestConfirm_t mib;

	if (val == NULL){
		return AT_PARAM_ERROR;
	}
	len = (size_t)(val - param);
	val++;
	while (*val == ' '){
		val++;
	}
	if (AT_parse_hex(val, key, 16U) != 16){
		return AT_PARAM_ERROR;
	}

	if ((len == 7U) && (strncasecmp(param, "NWKSKEY", 7) == 0)){
		name = "NWKSKEY";
		mib.Type = MIB_NWK_S_KEY;
		mib.Param.NwkSKey = key;
	}
	else if ((len == 7U) && (strncasecmp(param, "APPSKEY", 7) == 0)){
		name = "APPSKEY";
		mib.Type = MIB_APP_S_KEY;
		mib.Param.AppSKey = key;
	}
	else{
		return AT_PARAM_ERROR;
	}

	/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
	if (LoRaMacIsBusy()){
		return AT_BUSY_ERROR;
	}
	if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK){
		return AT_ERROR;
	}

	for (i = 0; i < 16U; i++){
		snprintf(&hex[2U * i], 3, "%02X", key[i]);
	}
	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+KEY: %s %s\r\n", name, hex);
	return AT_OK;
}

/* AT+ID=DevAddr, "<8 hex>" -> +ID: DevAddr, XX:XX:XX:XX
   MIB_DEV_ADDR -> SecureElementSetDevAddr() (DevAddrABP) + Nvm.MacGroup2.DevAddr; works also when
   activated, LmHandlerSetDevAddr() refuses it after the ABP activation. */
ATEerror_t AT_id_set(const char *param){
	const char *val = strchr(param, ',');
	uint8_t id[4];
	MibRequestConfirm_t mib;

	if ((val == NULL) || ((size_t)(val - param) != 7U) || (strncasecmp(param, "DevAddr", 7) != 0)){
		return AT_PARAM_ERROR;
	}
	val++;
	while (*val == ' '){
		val++;
	}
	if (AT_parse_hex(val, id, 4U) != 4){
		return AT_PARAM_ERROR;
	}

	/* LoRaMacIsBusy(), not LmHandlerIsBusy(): the latter starts a join when not activated */
	if (LoRaMacIsBusy()){
		return AT_BUSY_ERROR;
	}
	mib.Type = MIB_DEV_ADDR;
	mib.Param.DevAddr = ((uint32_t)id[0] << 24) | ((uint32_t)id[1] << 16) |
						((uint32_t)id[2] << 8) | (uint32_t)id[3];
	if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK){
		return AT_ERROR;
	}

	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+ID: DevAddr, %02X:%02X:%02X:%02X\r\n", id[0], id[1], id[2], id[3]);
	return AT_OK;
}

/* AT+MSGHEX / AT+CMSGHEX uplink in progress: "MSGHEX" | "CMSGHEX", NULL = none.
   Finished in AT_msghex_done() (OnTxData -> AT_event_confirm) */
static const char *msghex_name = NULL;
/* NbTrans (AT+REPT) replaced by AT+RETRY during a confirmed uplink, restored in AT_msghex_done().
   NbTrans lives in the NVM context: a context stored with the RETRY value would lose REPT */
static uint8_t msghex_rept;

/* AT+MSGHEX="<hex>"  -> +MSGHEX: Start ... +MSGHEX: Done
   AT+CMSGHEX="<hex>" -> +CMSGHEX: Start, +CMSGHEX: Wait ACK ... [+CMSGHEX: ACK Received], +CMSGHEX: Done
   FPort = AT+PORT; repetitions: unconfirmed = NbTrans (AT+REPT), confirmed = AT+RETRY.
   An immediate failure prints the Seeed error line without Done (the ESP driver takes Done as success). */
static ATEerror_t AT_msghex_send(const char *param, LmHandlerMsgTypes_t type){
	const char *name = (type == LORAMAC_HANDLER_CONFIRMED_MSG) ? "CMSGHEX" : "MSGHEX";
	int16_t size;
	uint8_t nb;
	LoRaMacTxInfo_t txInfo;
	MibRequestConfirm_t mib;
	LmHandlerErrorStatus_t status;

	if (ram_vars.mode != 0U){          /* TEST mode: LoRaMac stopped */
		return AT_ERROR;
	}
	if ((msghex_name != NULL) || LoRaMacIsBusy()){
		AT_PPRINTF("+%s: LoRaWAN modem is busy\r\n", name);
		return AT_BUSY_ERROR;
	}
	size = AT_parse_hex(param, AppData.Buffer, LORAWAN_APP_DATA_BUFFER_MAX_SIZE);
	if (size < 0){
		return AT_PARAM_ERROR;
	}
	/* LmHandlerSend() would send an empty frame instead of a too long payload */
	if (LoRaMacQueryTxPossible((uint8_t)size, &txInfo) != LORAMAC_STATUS_OK){
		AT_PPRINTF("+%s: Length error %u\r\n", name, (unsigned)txInfo.MaxPossibleApplicationDataSize);
		return AT_PARAM_ERROR;
	}
	AppData.BufferSize = (uint8_t)size;
	AppData.Port = eeprom_vars.port;

	if (type == LORAMAC_HANDLER_CONFIRMED_MSG){
		/* RETRY -> NbTrans: <2 = 1 transmission, stack limit 15 */
		nb = eeprom_vars.retry;
		if (nb < 1U){
			nb = 1U;
		}
		if (nb > 15U){
			nb = 15U;
		}
		mib.Type = MIB_CHANNELS_NB_TRANS;
		LoRaMacMibGetRequestConfirm(&mib);
		msghex_rept = mib.Param.ChannelsNbTrans;
		mib.Param.ChannelsNbTrans = nb;
		LoRaMacMibSetRequestConfirm(&mib);
	}

	AT_PPRINTF("+%s: Start\r\n", name);
	msghex_name = name;
	status = LmHandlerSend(&AppData, type, false);
	if (status == LORAMAC_HANDLER_SUCCESS){
		if (type == LORAMAC_HANDLER_CONFIRMED_MSG){
			AT_PPRINTF("+%s: Wait ACK\r\n", name);
		}
		return AT_OK;
	}

	/* Not sent: no OnTxData will come */
	msghex_name = NULL;
	if (type == LORAMAC_HANDLER_CONFIRMED_MSG){
		mib.Type = MIB_CHANNELS_NB_TRANS;
		mib.Param.ChannelsNbTrans = msghex_rept;
		LoRaMacMibSetRequestConfirm(&mib);
	}
	switch (status){
	case LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED:
		AT_PPRINTF("+%s: No band in %ums\r\n", name, (unsigned)LmHandlerGetDutyCycleWaitTime());
		break;
	case LORAMAC_HANDLER_NO_NETWORK_JOINED:
		AT_PPRINTF("+%s: Please join network first\r\n", name);
		break;
	case LORAMAC_HANDLER_BUSY_ERROR:
		AT_PPRINTF("+%s: LoRaWAN modem is busy\r\n", name);
		break;
	default:
		/* LmHandlerSend() maps LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND (LBT, AT+LW=THLD) to LORAMAC_HANDLER_ERROR */
		AT_PPRINTF("+%s: No free channel\r\n", name);
		break;
	}
	return AT_ERROR;
}

ATEerror_t AT_msghex_set(const char *param){
	return AT_msghex_send(param, LORAMAC_HANDLER_UNCONFIRMED_MSG);
}

ATEerror_t AT_cmsghex_set(const char *param){
	return AT_msghex_send(param, LORAMAC_HANDLER_CONFIRMED_MSG);
}

/* Called from AT_event_confirm() (OnTxData) when the uplink, including all NbTrans repetitions, is finished */
void AT_msghex_done(LmHandlerTxParams_t *params){
	MibRequestConfirm_t mib;

	if ((msghex_name == NULL) || (params->IsMcpsConfirm == 0U)){
		return;
	}
	if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG){
		mib.Type = MIB_CHANNELS_NB_TRANS;
		mib.Param.ChannelsNbTrans = msghex_rept;
		LoRaMacMibSetRequestConfirm(&mib);
		if (params->AckReceived != 0U){
			AT_PPRINTF("+%s: ACK Received\r\n", msghex_name);
		}
		else{
			/* all RETRY transmissions without ACK (McpsConfirm.Status = RX2 timeout/error):
			   no Done, the ESP driver takes Done as success and would not retry */
			AT_PPRINTF("+%s: No ACK\r\n", msghex_name);
			msghex_name = NULL;
			return;
		}
	}
	else if (params->Status != LORAMAC_EVENT_INFO_STATUS_OK){
		/* unconfirmed: TX timeout / error (an unconfirmed uplink without downlink keeps the status OK) */
		AT_PPRINTF("+%s: ERROR\r\n", msghex_name);
		msghex_name = NULL;
		return;
	}
	AT_PPRINTF("+%s: Done\r\n", msghex_name);
	msghex_name = NULL;
}

/* AT+REPT=<1..15> -> +REPT: <n>
   Transmissions of an unconfirmed uplink = NbTrans (MIB_CHANNELS_NB_TRANS, Nvm.MacGroup2.MacParams,
   LoRaWAN NVM context). With ADR on, the server (LinkADRReq NbRep) and the ADR backoff may change it. */
ATEerror_t AT_rept_set(const char *param){
	char *end;
	unsigned long rept = strtoul(param, &end, 10);
	MibRequestConfirm_t mib;

	if ((end == param) || (*end != '\0') || (rept < 1UL) || (rept > 15UL)){
		return AT_PARAM_ERROR;
	}
	/* During AT+CMSGHEX NbTrans holds AT+RETRY, AT_msghex_done() would overwrite the new value */
	if ((msghex_name != NULL) || LoRaMacIsBusy()){
		return AT_BUSY_ERROR;
	}
	mib.Type = MIB_CHANNELS_NB_TRANS;
	mib.Param.ChannelsNbTrans = (uint8_t)rept;
	if (LoRaMacMibSetRequestConfirm(&mib) != LORAMAC_STATUS_OK){
		return AT_ERROR;
	}
	nvm_dirty = 1U;      /* LoRaWAN NVM context changed: stored at AT+LW=NET,OFF / AT+NVM / AT+RESET */
	AT_PPRINTF("+REPT: %u\r\n", (unsigned)rept);
	return AT_OK;
}

/* AT+TEMP -> +TEMP: <°C>   (Seeed 4.25: one decimal; the internal sensor gives whole degrees
   (SYS_GetTemperatureLevel(), __LL_ADC_CALC_TEMPERATURE), the decimal is always 0) */
ATEerror_t AT_temp_run(const char *param){
	AT_PPRINTF("+TEMP: %d.0\r\n", (int)GetTemperatureLevel());
	return AT_OK;
}

/* AT+RTC="YYYY-MM-DD HH:MM:SS" -> +RTC: YYYY-MM-DD HH:MM:SS     (Seeed 4.24, year 20xx)
   AT+RTC=UTCMS                 -> +RTC: YYYY-MM-DD HH:MM:SS.mmm
   UTC kept by SysTime: RTC binary counter + offset in the RTC backup registers. SysTimeSet() changes
   only the offset, the counter used by the LoRaMac timers (UTIL_TIMER) is not touched. */
ATEerror_t AT_rtc_set(const char *param){
	const char sep[6] = { '-', '-', ' ', ':', ':', '"' };   /* character after each field */
	const char *p = param;
	char *end;
	unsigned long v[6];      /* year, month, day, hour, minute, second */
	uint8_t i;
	struct tm t = {0};
	SysTime_t st;

	if (strcasecmp(param, "UTCMS") == 0){
		st = SysTimeGet();
		SysTimeLocalTime(st.Seconds, &t);
		AT_PPRINTF("+RTC: %04d-%02d-%02d %02d:%02d:%02d.%03d\r\n", t.tm_year + 1900, t.tm_mon + 1,
				t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, (int)st.SubSeconds);
		return AT_OK;
	}

	if (*p != '"'){
		return AT_PARAM_ERROR;
	}
	p++;
	for (i = 0; i < 6U; i++){
		v[i] = strtoul(p, &end, 10);
		if ((end == p) || (*end != sep[i])){
			return AT_PARAM_ERROR;
		}
		p = end + 1;
	}
	if ((*p != '\0') || (v[0] < 2000UL) || (v[0] > 2099UL) || (v[1] < 1UL) || (v[1] > 12UL) ||
		(v[2] < 1UL) || (v[2] > 31UL) || (v[3] > 23UL) || (v[4] > 59UL) || (v[5] > 59UL)){
		return AT_PARAM_ERROR;
	}

	t.tm_year = (int)v[0] - 1900;
	t.tm_mon = (int)v[1] - 1;
	t.tm_mday = (int)v[2];
	t.tm_hour = (int)v[3];
	t.tm_min = (int)v[4];
	t.tm_sec = (int)v[5];
	st.Seconds = SysTimeMkTime(&t);
	st.SubSeconds = 0;
	SysTimeSet(st);

	/* echo of the parsed value: a read back could already be one second later */
	AT_PPRINTF("+RTC: %04u-%02u-%02u %02u:%02u:%02u\r\n", (unsigned)v[0], (unsigned)v[1], (unsigned)v[2],
			(unsigned)v[3], (unsigned)v[4], (unsigned)v[5]);    /* tiny_vsnprintf_like: no %l */
	return AT_OK;
}

/* AT+RTC? | AT+RTC -> +RTC: YYYY-MM-DD HH:MM:SS */
ATEerror_t AT_rtc_get(const char *param){
	struct tm t;
	SysTime_t st = SysTimeGet();

	SysTimeLocalTime(st.Seconds, &t);
	AT_PPRINTF("+RTC: %04d-%02d-%02d %02d:%02d:%02d\r\n", t.tm_year + 1900, t.tm_mon + 1,
			t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
	return AT_OK;
}

static ATEerror_t  AT_eeprom_get(const uint8_t v_address){
	uint8_t data = 0U;
	uint8_t user_v_address = v_address + EE_VA_USER_BASE;
	EE_Status st = EE_ReadVariable8bits(user_v_address, &data);

	/* never written (EE_NO_DATA): 00, the ESP driver reads the last sync timestamp from a fresh module */
	if (st == EE_NO_DATA){
		data = 0U;
	}
	else if (st != EE_OK){
		return AT_ERROR;
	}
	AT_PPRINTF("+EEPROM: %02X, %02X\r\n", v_address, data);
	return AT_OK;
}
ATEerror_t  AT_eeprom_set(const char *param){
	char* end;
	const char* p = param;
	unsigned long addr;
	unsigned long val;
	uint8_t v_address;
	uint8_t user_v_address;
	uint8_t data;

	// Parse address (range checked before the cast to uint8_t)
	addr = strtoul(p, &end, 16);
	if ((end == p) || (addr > (unsigned long)(EE_VA_USER_MAX - EE_VA_USER_BASE))){
		return AT_PARAM_ERROR;
	}
	v_address = (uint8_t)addr;
	user_v_address = v_address + EE_VA_USER_BASE;

	// Get data from address: AT+EEPROM=<addr>
	if (*end == '\0'){
		return AT_eeprom_get(v_address);
	}
	// Write data on address: AT+EEPROM=<addr>,<val>
	if (*end != ','){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	val = strtoul(p, &end, 16);
	if ((end == p) || (*end != '\0') || (val > 0xFFUL)){
		return AT_PARAM_ERROR;
	}
	data = (uint8_t)val;

	if (ee_write(user_v_address, data) != 0){
		return AT_ERROR;
	}
	AT_PPRINTF("+EEPROM: %02X, %02X\r\n", v_address, data);
	return AT_OK;
}

/* AT+TEST=RFCFG,<MHz>,SF<7..12>,<125|250|500>,<TXPR>,<RXPR>,<dBm>,ON,OFF,OFF
   -> +TEST: RFCFG F:<Hz>, SF<sf>, BW<bw>K, TXPR:<n>, RXPR:<n>, POW:<dBm>dBm, CRC:ON, IQ:OFF, NET:OFF
   Only the ESP driver setting: CRC ON, IQ OFF, NET OFF. F, SF, BW, power -> testParam (TST_set_config);
   preambles are fixed in test_rf.c (TST_LR_TX/RX_PREAMBLE), TXPR/RXPR are only echoed. */
static ATEerror_t AT_test_rfcfg_set(const char *p){
	char *end;
	uint32_t freq;
	unsigned long sf, bw, txpr, rxpr;
	long pow;
	testParameter_t tp;

	if (!AT_ch_parse_mhz(&p, &freq) || (*p != ',') || !AT_rf_freq_valid(freq)){
		return AT_PARAM_ERROR;
	}
	p++;
	if (strncasecmp(p, "SF", 2) == 0){
		p += 2;
	}
	sf = strtoul(p, &end, 10);
	if ((end == p) || (*end != ',') || (sf < 7UL) || (sf > 12UL)){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	bw = strtoul(p, &end, 10);
	if ((end == p) || (*end != ',') || ((bw != 125UL) && (bw != 250UL) && (bw != 500UL))){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	txpr = strtoul(p, &end, 10);
	if ((end == p) || (*end != ',')){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	rxpr = strtoul(p, &end, 10);
	if ((end == p) || (*end != ',')){
		return AT_PARAM_ERROR;
	}
	p = end + 1;
	pow = strtol(p, &end, 10);
	if ((end == p) || (pow < -9L) || (pow > 22L) || (strcasecmp(end, ",ON,OFF,OFF") != 0)){
		return AT_PARAM_ERROR;
	}

	TST_get_config(&tp);
	tp.modulation = TEST_LORA;
	tp.freq = freq;
	tp.power = (int32_t)pow;
	tp.bandwidth = (bw == 125UL) ? 4U : ((bw == 250UL) ? 5U : 6U);   /* test_rf.h: 4 = 125, 5 = 250, 6 = 500 kHz */
	tp.loraSf_datarate = (uint32_t)sf;
	tp.codingRate = 1U;                                              /* 4/5 */
	tp.lowDrOpt = 2U;                                                /* auto */
	TST_set_config(&tp);

	/* tiny_vsnprintf_like: no %l */
	AT_PPRINTF("+TEST: RFCFG F:%u, SF%u, BW%uK, TXPR:%u, RXPR:%u, POW:%ddBm, CRC:ON, IQ:OFF, NET:OFF\r\n",
			(unsigned)freq, (unsigned)sf, (unsigned)bw, (unsigned)txpr, (unsigned)rxpr, (int)pow);
	return AT_OK;
}

/* AT+TEST=TXLRPKT,<hex> -> +TEST: TXLRPKT "<HEX>", +TEST: TX DONE   (blocking, ~30 ms at SF7/BW500) */
static ATEerror_t AT_test_txlrpkt(const char *param){
	uint8_t data[255];
	int16_t len;
	int16_t i;

	if (ram_vars.mode != 1U){                 /* only in AT+MODE=TEST */
		return AT_ERROR;
	}
	len = AT_parse_hex(param, data, sizeof(data));
	if (len <= 0){
		return AT_PARAM_ERROR;
	}
	AT_PPRINTF("+TEST: TXLRPKT \"");
	for (i = 0; i < len; i++){
		AT_PPRINTF("%02X", data[i]);
	}
	AT_PPRINTF("\"\r\n");

	if (TST_TxLoraPacket(data, (uint8_t)len) != 0){
		AT_PPRINTF("+TEST: TX TIMEOUT\r\n");
		return AT_ERROR;
	}
	AT_PPRINTF("+TEST: TX DONE\r\n");
	return AT_OK;
}

/* AT+TEST=RXLRPKT -> +TEST: RXLRPKT, then continuous RX: every packet -> +TEST: LEN.., +TEST: RX "<HEX>"
   (TST_RxPrint in test_rf.c). Ends with AT+MODE=LWABP (TST_stop) or AT+TEST=TXLRPKT */
static ATEerror_t AT_test_rxlrpkt(void){
	if (ram_vars.mode != 1U){                 /* only in AT+MODE=TEST */
		return AT_ERROR;
	}
	TST_RxLoraContinuous();
	AT_PPRINTF("+TEST: RXLRPKT\r\n");
	return AT_OK;
}

/* AT+TEST=RFCFG,... | TXLRPKT,<hex> | RXLRPKT   (Seeed 4.31, only what the ESP driver uses: LoRa beacon) */
ATEerror_t AT_test_set(const char *param){
	if (strncasecmp(param, "RFCFG,", 6) == 0){
		return AT_test_rfcfg_set(param + 6);
	}
	if (strncasecmp(param, "TXLRPKT,", 8) == 0){
		return AT_test_txlrpkt(param + 8);
	}
	if (strcasecmp(param, "RXLRPKT") == 0){
		return AT_test_rxlrpkt();
	}
	return AT_PARAM_ERROR;
}

ATEerror_t AT_lowpower_set(const char *param){
	if (strcasecmp(param, "AUTOON") == 0){
		ram_vars.lowpower_auto = 1;
		AT_PPRINTF("+LOWPOWER: AUTOON\r\n");
		return AT_OK;
	}
	else if (strcasecmp(param, "AUTOOFF") == 0){
		ram_vars.lowpower_auto = 0;
		AT_PPRINTF("+LOWPOWER: AUTOOFF\r\n");
		return AT_OK;
	}
	return AT_PARAM_ERROR;
}
ATEerror_t AT_lowpower_run(const char *param){
	ram_vars.lowpower_once = 1;
	AT_PPRINTF("+LOWPOWER: SLEEP\r\n");
	return AT_OK;
}

/* AT+UART=BR,<baud>     -> +UART: BR, <baud>      (EEPROM, takes effect after a reset as Seeed;
                                                    the ESP driver sends AT+RESET right after it, usart.c)
   AT+UART=TIMEOUT,<ms>  -> +UART: TIMEOUT, <ms>   (EEPROM, 0 = off; an incomplete command is discarded
                                                    after <ms> without a new character, lora_command.c)
   AT+UART=BR | TIMEOUT  -> current value */
ATEerror_t AT_uart_set(const char *param){
	const char *val = strchr(param, ',');
	size_t len = (val != NULL) ? (size_t)(val - param) : strlen(param);
	char *end;
	unsigned long v;

	if ((len == 2U) && (strncasecmp(param, "BR", 2) == 0)){
		if (val != NULL){
			val++;
			while (*val == ' '){
				val++;
			}
			v = strtoul(val, &end, 10);
			if ((end == val) || (*end != '\0') ||
				((v != 9600UL) && (v != 14400UL) && (v != 19200UL) && (v != 38400UL) &&
				 (v != 57600UL) && (v != 76800UL) && (v != 115200UL) && (v != 230400UL))){
				return AT_PARAM_ERROR;
			}
			if (eeprom_set_uart_baud((uint32_t)v) != EE_OK){
				return AT_ERROR;
			}
		}
		AT_PPRINTF("+UART: BR, %u\r\n", (unsigned)eeprom_vars.uart_baud);
		return AT_OK;
	}
	if ((len == 7U) && (strncasecmp(param, "TIMEOUT", 7) == 0)){
		if (val != NULL){
			val++;
			while (*val == ' '){
				val++;
			}
			v = strtoul(val, &end, 10);
			if ((end == val) || (*end != '\0') || (v > 65535UL)){
				return AT_PARAM_ERROR;
			}
			if (eeprom_set_uart_timeout((uint16_t)v) != EE_OK){
				return AT_ERROR;
			}
		}
		AT_PPRINTF("+UART: TIMEOUT, %u\r\n", (unsigned)eeprom_vars.uart_timeout_ms);
		return AT_OK;
	}
	return AT_PARAM_ERROR;
}

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
static int32_t sscanf_uint32_as_hhx(const char *from, uint32_t *value)
{
  /* USER CODE BEGIN sscanf_uint32_as_hhx_1 */

  /* USER CODE END sscanf_uint32_as_hhx_1 */
  return tiny_sscanf(from, "%hhx:%hhx:%hhx:%hhx",
                     &((unsigned char *)(value))[3],
                     &((unsigned char *)(value))[2],
                     &((unsigned char *)(value))[1],
                     &((unsigned char *)(value))[0]);
  /* USER CODE BEGIN sscanf_uint32_as_hhx_2 */

  /* USER CODE END sscanf_uint32_as_hhx_2 */
}

static int32_t sscanf_16_hhx(const char *from, uint8_t *pt)
{
  /* USER CODE BEGIN sscanf_16_hhx_1 */

  /* USER CODE END sscanf_16_hhx_1 */
  return tiny_sscanf(from, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                     &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6],
                     &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13],
                     &pt[14], &pt[15]);
  /* USER CODE BEGIN sscanf_16_hhx_2 */

  /* USER CODE END sscanf_16_hhx_2 */
}

static void print_uint32_as_02x(uint32_t value)
{
  /* USER CODE BEGIN print_uint32_as_02x_1 */

  /* USER CODE END print_uint32_as_02x_1 */
  AT_PPRINTF("%02X:%02X:%02X:%02X\r\n",
            (unsigned)((unsigned char *)(&value))[3],
            (unsigned)((unsigned char *)(&value))[2],
            (unsigned)((unsigned char *)(&value))[1],
            (unsigned)((unsigned char *)(&value))[0]);
  /* USER CODE BEGIN print_uint32_as_02x_2 */

  /* USER CODE END print_uint32_as_02x_2 */
}

static void print_16_02x(uint8_t *pt)
{
  /* USER CODE BEGIN print_16_02x_1 */

  /* USER CODE END print_16_02x_1 */
  AT_PPRINTF("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            pt[0], pt[1], pt[2], pt[3],
            pt[4], pt[5], pt[6], pt[7],
            pt[8], pt[9], pt[10], pt[11],
            pt[12], pt[13], pt[14], pt[15]);
  /* USER CODE BEGIN print_16_02x_2 */

  /* USER CODE END print_16_02x_2 */
}

static void print_8_02x(uint8_t *pt)
{
  /* USER CODE BEGIN print_8_02x_1 */

  /* USER CODE END print_8_02x_1 */
  AT_PPRINTF("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
  /* USER CODE BEGIN print_8_02x_2 */

  /* USER CODE END print_8_02x_2 */
}

static void print_d(int32_t value)
{
  /* USER CODE BEGIN print_d_1 */

  /* USER CODE END print_d_1 */
  AT_PPRINTF("%d\r\n", value);
  /* USER CODE BEGIN print_d_2 */

  /* USER CODE END print_d_2 */
}

static void print_u(uint32_t value)
{
  /* USER CODE BEGIN print_u_1 */

  /* USER CODE END print_u_1 */
  AT_PPRINTF("%u\r\n", value);
  /* USER CODE BEGIN print_u_2 */

  /* USER CODE END print_u_2 */
}

static void OnCertifTimer(void *context)
{
  /* USER CODE BEGIN OnCertifTimer_1 */

  /* USER CODE END OnCertifTimer_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaCertifTx), CFG_SEQ_Prio_0);
  /* USER CODE BEGIN OnCertifTimer_2 */

  /* USER CODE END OnCertifTimer_2 */
}

static void CertifSend(void)
{
  /* USER CODE BEGIN CertifSend_1 */

  /* USER CODE END CertifSend_1 */
  AppData.Buffer[0] = 0x43;
  AppData.BufferSize = 1;
  AppData.Port = 99;

  /* Restart Tx to prevent a previous Join Failed */
  if (LmHandlerJoinStatus() != LORAMAC_HANDLER_SET)
  {
    UTIL_TIMER_Start(&TxCertifTimer);
  }
  LmHandlerSend(&AppData, LORAMAC_HANDLER_UNCONFIRMED_MSG, false);
}

static uint8_t Char2Nibble(char Char)
{
  if (((Char >= '0') && (Char <= '9')))
  {
    return Char - '0';
  }
  else if (((Char >= 'a') && (Char <= 'f')))
  {
    return Char - 'a' + 10;
  }
  else if ((Char >= 'A') && (Char <= 'F'))
  {
    return Char - 'A' + 10;
  }
  else
  {
    return 0xF0;
  }
  /* USER CODE BEGIN CertifSend_2 */

  /* USER CODE END CertifSend_2 */
}

static int32_t stringToData(const char *str, uint8_t *data, uint32_t Size)
{
  /* USER CODE BEGIN stringToData_1 */

  /* USER CODE END stringToData_1 */
  char hex[3];
  hex[2] = 0;
  int32_t ii = 0;
  while (Size-- > 0)
  {
    hex[0] = *str++;
    hex[1] = *str++;

    /*check if input is hex */
    if ((isHex(hex[0]) == -1) || (isHex(hex[1]) == -1))
    {
      return -1;
    }
    /*check if input is even nb of character*/
    if ((hex[1] == '\0') || (hex[1] == ','))
    {
      return -1;
    }
    data[ii] = (Char2Nibble(hex[0]) << 4) + Char2Nibble(hex[1]);

    ii++;
  }

  return 0;
  /* USER CODE BEGIN stringToData_2 */

  /* USER CODE END stringToData_2 */
}

static int32_t isHex(char Char)
{
  /* USER CODE BEGIN isHex_1 */

  /* USER CODE END isHex_1 */
  if (((Char >= '0') && (Char <= '9')) ||
      ((Char >= 'a') && (Char <= 'f')) ||
      ((Char >= 'A') && (Char <= 'F')))
  {
    return 0;
  }
  else
  {
    return -1;
  }
  /* USER CODE BEGIN isHex_2 */

  /* USER CODE END isHex_2 */
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
