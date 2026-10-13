/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   Independent watchdog (IWDG) configuration and refresh
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
/**
  * @brief IWDG timeout (LSI ~32 kHz, prescaler 256, reload 4095 => ~32.8 s)
  */
#define IWDG_PRESCALER_VALUE   IWDG_PRESCALER_256
#define IWDG_RELOAD_VALUE      4095U

/**
  * @brief Period (ms) of the periodic timer that refreshes the IWDG.
  *        It wakes the MCU from Stop mode, must be well below the IWDG timeout.
  */
#define IWDG_REFRESH_PERIOD_MS 10000U

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Start the IWDG and the periodic refresh timer.
  * @note   Call after SystemApp_Init() (UTIL_TIMER must be initialized).
  *         The IWDG cannot be stopped once started (only by reset).
  */
void MX_IWDG_Init(void);

/**
  * @brief  Refresh (kick) the IWDG
  */
void IWDG_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */
