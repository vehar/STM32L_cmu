//battery
#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32l1xx.h"
#include "adc.h"
#include "ina219.h"
#include <stdbool.h> 

bool Batt_voltage_check(void);
bool Batt_current_check(void);

#endif //__BATTERY_H