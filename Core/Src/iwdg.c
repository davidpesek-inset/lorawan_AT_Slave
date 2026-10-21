/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   Independent watchdog (IWDG) configuration and refresh
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "lora_at.h"
#include "sys_app.h"

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

static UTIL_TIMER_Object_t IwdgRefreshTimer;

/* Private function prototypes -----------------------------------------------*/
static void OnIwdgRefreshTimerEvent(void *context);
static void IwdgRefreshTask(void);

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

  /* The MCU sleeps in Stop mode most of the time: the timer wakes it up periodically and schedules
     the refresh task. The refresh itself runs in the main loop (sequencer), so a stuck main loop
     or a task that never returns is caught by the IWDG */
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_IwdgRefresh), UTIL_SEQ_RFU, IwdgRefreshTask);
  UTIL_TIMER_Create(&IwdgRefreshTimer, IWDG_REFRESH_PERIOD_MS, UTIL_TIMER_PERIODIC, OnIwdgRefreshTimerEvent, NULL);
  UTIL_TIMER_Start(&IwdgRefreshTimer);
  APP_LOG(TS_OFF, VLEVEL_H, "WDT init\r\n");
}

// refresh pres zapis do registru muze byt volan i kdyz neni watchdog inicializovan
void IWDG_Refresh(void)
{
	WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD);
}

/* Private functions ---------------------------------------------------------*/
/* Timer IRQ context: only schedule the refresh */
static void OnIwdgRefreshTimerEvent(void *context)
{
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_IwdgRefresh), CFG_SEQ_Prio_0);
}

/* CFG_SEQ_Task_IwdgRefresh: main loop context */
static void IwdgRefreshTask(void)
{
  IWDG_Refresh();
}
