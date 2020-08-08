#ifndef INO_2_HAL_H
#define INO_2_HAL_H

#include "main.h"

#define millis() HAL_GetTick()
#define delay(uint16_t val) osDelay(uint16_t val)


#endif
