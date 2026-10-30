/*
 * eeprom_wrapper.c
 *
 *  Created on: 14. 10. 2026
 *      Author: pesek
 */

#include "eeprom_wrapper.h"

_Static_assert((EE_VA_USER_BASE + EE_VA_USER_WORDS - 1U) <= NB_OF_VARIABLES, "VA must be 1..NB_OF_VARIABLES");

// DEFAULT SETTINGS BELOW ARE THE SETTINGS, THAT ARE SET DURING INITIALIZATION OF STM/LORA
#define EE_DEF_UART_BR          9600U
#define EE_DEF_UART_TIMEOUT     1000U
#define EE_DEF_DR               4U
#define EE_DEF_DC               1U
#define EE_DEF_NET              0U
#define EE_DEF_LDRO             0U
#define EE_DEF_THLD             (-65)
#define EE_DEF_DUMRX            1U
#define EE_DEF_PORT             8U
#define EE_DEF_RETRY            1U
#define EE_DEF_WDT				1U
#define EE_DEF_CT               0U

EepromVars_t eeprom_vars =
{
  .uart_baud       = EE_DEF_UART_BR,
  .uart_timeout_ms = EE_DEF_UART_TIMEOUT,
  .dr              = EE_DEF_DR,
  .dc              = EE_DEF_DC,
  .net             = EE_DEF_NET,
  .ldro            = EE_DEF_LDRO,
  .thld_dbm        = EE_DEF_THLD,
  .dumrx           = EE_DEF_DUMRX,
  .port            = EE_DEF_PORT,
  .retry           = EE_DEF_RETRY,
  .wdt			   = EE_DEF_WDT,
  .ct              = EE_DEF_CT,
};

/* TODO: verify the test RF defaults on the original module */

RamVars_t ram_vars =
{
  .mode          = 0U,              /* 0:LWABP, 1:TEST */
  .log_level     = 0U,              /* QUIET */
  .lowpower_auto = 1U,              /* AUTOON */
  .lowpower_once = 0U,
  .adr			 = 0U,
  .class		 = 0U,
  .rx1_delay	 = 1U,
  .rx2_delay	 = 10U,
  .rx1_win		 = 0U,
  .rx2_win		 = 0U,
  .ch_data		 = {
		  [0] = { 863800000U, 0U, 5U },
		  [1] = { 864000000U, 0U, 5U },
		  [2] = { 864200000U, 0U, 5U },
		  [3] = { 864400000U, 0U, 5U },
		  [4] = { 864600000U, 0U, 5U },
		  [5] = { 864800000U, 0U, 5U },
		  [6] = { 865000000U, 0U, 5U },
		  [7] = { 865200000U, 0U, 5U },
  },
  .test_freq_hz  = 868000000U,
  .test_sf       = 12U,
  .test_bw_khz   = 125U,
  .test_txpr     = 8U,
  .test_rxpr     = 8U,
  .test_pow_dbm  = 14,
  .test_crc      = 1U,
  .test_iq       = 0U,
  .test_net      = 0U,
};

/* Writes one variable only when the stored value differs.
   EE_CleanUp() erases flash pages (CPU stalls): do not call during a LoRaWAN TX/RX window. */
EE_Status ee_write(uint16_t va, uint32_t data)
{
  uint32_t stored;
  EE_Status status;

  if ((EE_ReadVariable32bits(va, &stored) == EE_OK) && (stored == data))
  {
    return EE_OK;
  }

  HAL_FLASH_Unlock();
  status = EE_WriteVariable32bits(va, data);
  if (status == EE_CLEANUP_REQUIRED)
  {
    status = EE_CleanUp();
  }
  HAL_FLASH_Lock();

  return status;
}

EE_Status eeprom_vars_init(void)
{
  uint32_t v;;
  /* First boot: no marker yet, store the defaults from eeprom_vars */
  if (EE_ReadVariable32bits(EE_VA_INIT_MARK, &v) == EE_NO_DATA)
  {
    return eeprom_vars_write_all();
  }
  /* A variable that is not found keeps its default value */
  if (EE_ReadVariable32bits(EE_VA_UART_BR, &v) == EE_OK)      { eeprom_vars.uart_baud       = v; }
  if (EE_ReadVariable32bits(EE_VA_UART_TIMEOUT, &v) == EE_OK) { eeprom_vars.uart_timeout_ms = (uint16_t)v; }
  if (EE_ReadVariable32bits(EE_VA_DR, &v) == EE_OK)           { eeprom_vars.dr              = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_DC, &v) == EE_OK)           { eeprom_vars.dc              = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_NET, &v) == EE_OK)          { eeprom_vars.net             = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_LDRO, &v) == EE_OK)         { eeprom_vars.ldro            = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_THLD, &v) == EE_OK)         { eeprom_vars.thld_dbm        = (int16_t)(uint16_t)v; }
  if (EE_ReadVariable32bits(EE_VA_DUMRX, &v) == EE_OK)        { eeprom_vars.dumrx           = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_PORT, &v) == EE_OK)         { eeprom_vars.port            = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_RETRY, &v) == EE_OK)        { eeprom_vars.retry           = (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_WDT, &v) == EE_OK)		  { eeprom_vars.wdt           	= (uint8_t)v; }
  if (EE_ReadVariable32bits(EE_VA_CT, &v) == EE_OK)           { eeprom_vars.ct              = (uint8_t)v; }
  return EE_OK;
}

EE_Status eeprom_vars_write_all(void)
{
  uint32_t status = EE_OK;     /* OR of all results: non-zero = at least one write failed */

  status |= ee_write(EE_VA_UART_BR,      eeprom_vars.uart_baud);
  status |= ee_write(EE_VA_UART_TIMEOUT, eeprom_vars.uart_timeout_ms);
  status |= ee_write(EE_VA_DR,           eeprom_vars.dr);
  status |= ee_write(EE_VA_DC,           eeprom_vars.dc);
  status |= ee_write(EE_VA_NET,          eeprom_vars.net);
  status |= ee_write(EE_VA_LDRO,         eeprom_vars.ldro);
  status |= ee_write(EE_VA_THLD,         (uint16_t)eeprom_vars.thld_dbm);
  status |= ee_write(EE_VA_DUMRX,        eeprom_vars.dumrx);
  status |= ee_write(EE_VA_PORT,         eeprom_vars.port);
  status |= ee_write(EE_VA_RETRY,        eeprom_vars.retry);
  status |= ee_write(EE_VA_WDT,			 eeprom_vars.wdt);
  status |= ee_write(EE_VA_CT,           eeprom_vars.ct);

  /* Marker last and only on success: an interrupted run is repeated at the next boot */
  if (status != EE_OK)
  {
    return EE_WRITE_ERROR;
  }
  return ee_write(EE_VA_INIT_MARK, EE_INIT_MARK_VALUE);
}

EE_Status eeprom_set_uart_baud(uint32_t value)
{
  eeprom_vars.uart_baud = value;
  return ee_write(EE_VA_UART_BR, value);
}

EE_Status eeprom_set_uart_timeout(uint16_t value)
{
  eeprom_vars.uart_timeout_ms = value;
  return ee_write(EE_VA_UART_TIMEOUT, value);
}

EE_Status eeprom_set_dr(uint8_t value)
{
  eeprom_vars.dr = value;
  return ee_write(EE_VA_DR, value);
}

EE_Status eeprom_set_dc(uint8_t value)
{
  eeprom_vars.dc = value;
  return ee_write(EE_VA_DC, value);
}

EE_Status eeprom_set_net(uint8_t value)
{
  eeprom_vars.net = value;
  return ee_write(EE_VA_NET, value);
}

EE_Status eeprom_set_ldro(uint8_t value)
{
  eeprom_vars.ldro = value;
  return ee_write(EE_VA_LDRO, value);
}

EE_Status eeprom_set_thld(int16_t value)
{
  eeprom_vars.thld_dbm = value;
  return ee_write(EE_VA_THLD, (uint16_t)value);   /* -65 is stored as 0x0000FFBF */
}

EE_Status eeprom_set_dumrx(uint8_t value)
{
  eeprom_vars.dumrx = value;
  return ee_write(EE_VA_DUMRX, value);
}

EE_Status eeprom_set_port(uint8_t value)
{
  eeprom_vars.port = value;
  return ee_write(EE_VA_PORT, value);
}

EE_Status eeprom_set_retry(uint8_t value)
{
  eeprom_vars.retry = value;
  return ee_write(EE_VA_RETRY, value);
}

EE_Status eeprom_set_wdt(uint8_t value)
{
  eeprom_vars.wdt = value;
  return ee_write(EE_VA_WDT, value);
}

EE_Status eeprom_set_ct(uint8_t value)
{
  eeprom_vars.ct = value;
  return ee_write(EE_VA_CT, value);
}
