/*
 * lorawan_functions.h
 *
 *  Created on: 27. 10. 2026
 *      Author: pesek
 */

#ifndef LORAWAN_APP_LORAWAN_FUNCTIONS_H_
#define LORAWAN_APP_LORAWAN_FUNCTIONS_H_





#include <stdint.h>
#include <stdbool.h>

typedef enum eLW_InitError
{
  LW_OK = 0,
  LW_ERROR = 1,
  LW_PARAM_ERROR = 2,
  LW_BUSY_ERROR = 3,
} LW_InitError_t;

/* Sub-GHz radio of the STM32WL: 150..960 MHz (RM0461). Radio.CheckRfFrequency() always returns true
   (radio.c), so the range is checked here */
#define LW_RF_FREQ_MIN_HZ   150000000UL
#define LW_RF_FREQ_MAX_HZ   960000000UL

bool lw_rf_freq_valid(uint32_t freq);

LW_InitError_t lw_nvm_get(LoRaMacNvmData_t** nvm);
LW_InitError_t lw_nvm_store(uint8_t force);
LW_InitError_t lw_mode_set(uint8_t mode);
LW_InitError_t lw_abp_deactivate(void);
LW_InitError_t lw_region_set(LoRaMacRegion_t regionSet);
LW_InitError_t lw_dr_set(uint8_t dr);
LW_InitError_t lw_ch_set(uint32_t id, uint32_t dr_min, uint32_t dr_max, uint32_t freq);
LW_InitError_t lw_class_set(DeviceClass_t class);
LW_InitError_t lw_adr_set(bool adr);
LW_InitError_t lw_power_set(long dbm, bool force);
LW_InitError_t lw_dc_set(bool enable);
LW_InitError_t lw_net_set(uint8_t public);
LW_InitError_t lw_ul_set(uint32_t ul);
LW_InitError_t lw_dl_set(uint32_t dl);
LW_InitError_t lw_ct_set(bool ct);
LW_InitError_t lw_dumrx_set(bool dumrx);
LW_InitError_t lw_thld_set(int16_t thld);
LW_InitError_t lw_port_set(uint8_t port);
LW_InitError_t lw_rx1Delay_set(uint32_t delay);
LW_InitError_t lw_rx2Delay_set(uint32_t delay);
LW_InitError_t lw_retry_set(uint8_t retry);
LW_InitError_t lw_rept_set(uint8_t rept);
LW_InitError_t lw_nwkskey_set(const char *key);
LW_InitError_t lw_appskey_set(const char *key);
LW_InitError_t lw_devaddr_set(const char *devaddr);

#endif /* LORAWAN_APP_LORAWAN_FUNCTIONS_H_ */
