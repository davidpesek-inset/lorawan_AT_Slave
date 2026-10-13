/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   Independent watchdog (IWDG) configuration and refresh
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
#include "stm32_timer.h"

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

static UTIL_TIMER_Object_t IwdgRefreshTimer;

/* Private function prototypes -----------------------------------------------*/
static void OnIwdgRefreshTimerEvent(void *context);

/* Exported functions --------------------------------------------------------*/
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_VALUE;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload = IWDG_RELOAD_VALUE;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

  /* The MCU sleeps in Stop mode most of the time: wake it up periodically to refresh the IWDG */
  UTIL_TIMER_Create(&IwdgRefreshTimer, IWDG_REFRESH_PERIOD_MS, UTIL_TIMER_PERIODIC, OnIwdgRefreshTimerEvent, NULL);
  UTIL_TIMER_Start(&IwdgRefreshTimer);
}

void IWDG_Refresh(void)
{
  HAL_IWDG_Refresh(&hiwdg);
}

/* Private functions ---------------------------------------------------------*/
static void OnIwdgRefreshTimerEvent(void *context)
{
  IWDG_Refresh();
}
