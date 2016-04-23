//device_power
#ifndef __DEVICE_POWER_H
#define __DEVICE_POWER_H

#include "stm32l1xx.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_gpio.h"

#include "board.h"
#include <stdbool.h>


void Proc_Pow_ON(void);
void Proc_Pow_OFF(void);

#endif //__DEVICE_POWER_H
