#include "battery.h"

bool Batt_voltage_check(void)
{
	uint32_t data = INA219_busVoltageRaw();
	if((data > AKK_VOLTAGE_MIN_RED)&&(data < AKK_VOLTAGE_MAX_RED)) 
		return true;
	else 
		return false; 
} 


bool Batt_current_check(void)
{ 
	uint32_t data = INA219_shuntCurrent(); //INA219_shuntCurrent_Raw
	if((data > AKK_CURRENT_MIN_RED)&&(data < AKK_CURRENT_MAX_RED)) 
		return true;
	else 
		return false;
}