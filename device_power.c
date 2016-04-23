#include "device_power.h"
#include "beeper.h"

extern bool f_PowActive;
extern bool f_SystemOk;

void Proc_Pow_ON(void)
{
 if(f_PowActive == false)
	{			
	f_PowActive = true;
		LED_ON;
	MAIN_RWR_ON;
	BeepDelauyed(10,10,5);	
	}
}

void Proc_Pow_OFF(void)
{
 if(f_PowActive == true)
	{			
	f_PowActive = false;
	BeepDelauyed(1,5,5);
	MAIN_RWR_OFF;	
		LED_OFF;
	}
}