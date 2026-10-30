/*
 * lorawan_hooks.c
 *
 *  Created on: 30. 10. 2026
 *      Author: pesek
 *
 *  Strong definitions of the weak hooks added to the LoRaWAN middleware
 *  (Libs/Middlewares/Third_Party/LoRaWAN). The stack calls them to read the AT settings
 *  stored in EEPROM; the weak defaults keep the original ST behaviour.
 *
 *  The file must be linked as an object file, not from a static library (.a): the linker
 *  does not pull an object from a library to replace a weak symbol, so the weak defaults
 *  would stay active without any warning. Check the .map file: all hooks must come from
 *  lorawan_hooks.o.
 */

#include <stdbool.h>
#include <stdint.h>
#include "eeprom_wrapper.h"
#include "LoRaMac.h"
#include "RegionEU868.h"

/* AT+LW=THLD (weak in RegionEU868.c): LBT RSSI threshold [dBm], 0 = LBT off */
int16_t RegionEU868LbtThreshold(void){
	return eeprom_vars.thld_dbm;
}

/* AT+LW=DUMRX (weak in LoRaMac.c): ON = no RX windows after an unconfirmed uplink */
bool LoRaMacSkipRxWindows(void){
	return (eeprom_vars.dumrx != 0U);
}

/* AT+LW=CT (weak in LoRaMac.c): OFF = any valid downlink in RX1/RX2
   terminates a confirmed uplink (Seeed relaxation 1) */
bool LoRaMacAckRelaxed(void){
	return (eeprom_vars.ct == 0U);
}
