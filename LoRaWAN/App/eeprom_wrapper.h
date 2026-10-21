/*
 * eeprom_wrapper.h
 *
 *  Created on: 14. 10. 2026
 *      Author: pesek
 */

#ifndef LORAWAN_APP_EEPROM_WRAPPER_H_
#define LORAWAN_APP_EEPROM_WRAPPER_H_

#include <stdint.h>
#include "eeprom_emul.h"

/* EEPROM virtual addresses (must be 1..NB_OF_VARIABLES) */
#define EE_VA_INIT_MARK         1U    /* first-boot marker, written last by eeprom_vars_write_all() */
#define EE_VA_UART_BR           2U
#define EE_VA_UART_TIMEOUT      3U
#define EE_VA_DR                4U
#define EE_VA_DC                5U
#define EE_VA_NET               6U
#define EE_VA_LDRO              7U
#define EE_VA_THLD              8U
#define EE_VA_DUMRX             9U
#define EE_VA_PORT              10U
#define EE_VA_RETRY             11U
#define EE_VA_WDT				12U
#define EE_VA_CT                13U
#define EE_VA_MODE				14U
/* 15..16 reserved */
#define EE_VA_USER_BASE         17U   /* AT+EEPROM user area, 4 bytes per variable */
#define EE_VA_USER_WORDS        96U   /* 70 x 4 B = 280 B, VA 17..86 */
#define EE_VA_USER_MIN			EE_VA_USER_BASE
#define EE_VA_USER_MAX			(EE_VA_USER_BASE + EE_VA_USER_WORDS - 1)
#define EE_INIT_MARK_VALUE      1U

/* Settings stored in EEPROM (RAM copy) */
typedef struct
{
  uint32_t uart_baud;        /* AT+UART=BR */
  uint16_t uart_timeout_ms;  /* AT+UART=TIMEOUT */
  uint8_t  dr;               /* AT+DR=<n> */
  uint8_t  dc;               /* AT+LW=DC     0 = OFF, 1 = ON */
  uint8_t  net;              /* AT+LW=NET    0 = OFF (private), 1 = ON (public) */
  uint8_t  ldro;             /* AT+LW=LDRO   0 = OFF, 1 = ON, 2 = AUTO */
  int16_t  thld_dbm;         /* AT+LW=THLD */
  uint8_t  dumrx;            /* AT+LW=DUMRX  0 = OFF, 1 = ON */
  uint8_t  port;             /* AT+PORT */
  uint8_t  retry;            /* AT+RETRY */
  uint8_t  wdt;            /* AT+WDT */
  uint8_t  ct;               /* AT+LW=CT     0 = OFF (Seeed relaxations, default), 1 = ON (strict LoRaWAN) */
  uint8_t  mode;			// AT+MODE=LWABP|TEST	0 = LWABP, 1 = TEST
} EepromVars_t;

typedef struct{
	uint32_t freq;
	uint8_t drmin;
	uint8_t drmax;
} LwChData_t;

/* Settings kept only in RAM (defaults at every boot) */
typedef struct
{
  uint8_t  mode;             /* AT+MODE      0 = LWABP, 1 = TEST */
  uint8_t  log_level;        /* AT+LOG       0 = QUIET, 1 = DEBUG */
  uint8_t  lowpower_auto;    /* AT+LOWPOWER  0 = AUTOOFF, 1 = AUTOON */
  volatile uint8_t lowpower_once;
  uint8_t  power_force;      /* AT+POWER     1 = FORCE */
  uint8_t  power;
  uint8_t  adr;
  uint8_t  class;
  uint8_t  ct;
  uint8_t  rx1_delay;
  uint8_t  rx2_delay;
  uint8_t  rx1_win;
  uint8_t  rx2_win;
  LwChData_t ch_data[17];
  uint32_t test_freq_hz;     /* AT+TEST=RFCFG */
  uint8_t  test_sf;
  uint16_t test_bw_khz;
  uint16_t test_txpr;
  uint16_t test_rxpr;
  int8_t   test_pow_dbm;
  uint8_t  test_crc;         /* 0 = OFF, 1 = ON */
  uint8_t  test_iq;          /* 0 = OFF, 1 = ON */
  uint8_t  test_net;         /* 0 = OFF, 1 = ON */
} RamVars_t;

extern EepromVars_t eeprom_vars;
extern RamVars_t    ram_vars;

EE_Status ee_write(uint16_t va, uint32_t data);
/* Boot: loads eeprom_vars from EEPROM. First boot (no init marker): writes the defaults. */
EE_Status eeprom_vars_init(void);

/* Writes the whole eeprom_vars to EEPROM, init marker last. */
EE_Status eeprom_vars_write_all(void);

/* Setters: RAM copy + EEPROM (EEPROM is written only when the value changes). */
EE_Status eeprom_set_uart_baud(uint32_t value);
EE_Status eeprom_set_uart_timeout(uint16_t value);
EE_Status eeprom_set_dr(uint8_t value);
EE_Status eeprom_set_dc(uint8_t value);
EE_Status eeprom_set_net(uint8_t value);
EE_Status eeprom_set_ldro(uint8_t value);
EE_Status eeprom_set_thld(int16_t value);
EE_Status eeprom_set_dumrx(uint8_t value);
EE_Status eeprom_set_port(uint8_t value);
EE_Status eeprom_set_retry(uint8_t value);
EE_Status eeprom_set_wdt(uint8_t value);
EE_Status eeprom_set_ct(uint8_t value);

#endif /* LORAWAN_APP_EEPROM_WRAPPER_H_ */
