/**
  ******************************************************************************
  * @file    eeprom_emul_conf.h
  * @brief   Configuration of the EEPROM emulation (X-CUBE-EEPROM 9.0.0, STM32WL port)
  *          for STM32WLE5JC (256 KB flash, 2 KB pages, 64-bit programming)
  *
  * Flash map (see STM32WLE5JCIX_FLASH.ld):
  *   0x08000000 - 0x08039FFF  firmware           (FLASH, 232 KB)
  *   0x0803A000 - 0x0803CFFF  EEPROM emulation   (6 pages, 12 KB)  <- this file
  *   0x0803E500 - 0x0803E7FF  user keys region
  *   0x0803F000 - 0x0803F7FF  LoRaWAN NVM        (page 126)
  *
  * Pages used = ((NB_OF_VARIABLES / 252, rounded up) * 2) * CYCLES_NUMBER + GUARD_PAGES_NUMBER
  *            = (2 * 2) * 1 + 2 = 6
  *
  * Usage rule: the library needs the flash unlocked (HAL_FLASH_Unlock()) while
  * EE_* functions run. The LoRaWAN FLASH_IF module unlocks/locks the flash
  * itself and treats an already unlocked flash as an error, so lock the flash
  * again (HAL_FLASH_Lock()) after the EE_* calls.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_EMUL_CONF_H
#define __EEPROM_EMUL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* DO NOT CHANGE the START_PAGE_ADDRESS without updating the linker script (FLASH length) */
#define START_PAGE_ADDRESS      0x0803A000U /*!< Start address of the 1st page in flash, for EEPROM emulation */

#define CYCLES_NUMBER           1U   /*!< Number of 10Kcycles requested, minimum 1 for 10Kcycles (default),
                                        for instance 10 to reach 100Kcycles. This factor will increase
                                        pages number */

#define GUARD_PAGES_NUMBER      2U   /*!< Number of guard pages avoiding frequent transfers (must be multiple of 2): 0,2,4.. */

/* Exported constants --------------------------------------------------------*/
/* CRC (hardware CRC peripheral, configured by EE_Init() through LL functions) */
#define CRC_POLYNOMIAL_LENGTH   LL_CRC_POLYLENGTH_16B /* CRC polynomial length 16 bits */
#define CRC_POLYNOMIAL_VALUE    0x8005U /* Polynomial to use for CRC calculation */

/* Number of variables to handle in eeprom: 256 bytes of user EEPROM + 4 settings */
#define NB_OF_VARIABLES         260U

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_EMUL_CONF_H */
