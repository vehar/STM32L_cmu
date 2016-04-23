#include "btns_handler.h"
#include "temperature_measurement.h"
#include "device_power.h"

extern uint32_t f_ButtPressed;

void butt_handler (void) //DODO: use http://chipenable.ru/index.php/programming-avr/item/218-biblioteka-dlya-oprosa-knopok.html
{
	if(BUTT_F_IS_SET(BUTT_1)) //1
	{
		LED_ON;			
		BeepDelauyed(1,1,2);
	}
	
	if(BUTT_F_IS_SET(BUTT_2)) //2
	{
		LED_ON;	
		STM_temperature_check();
	}
	
	if(BUTT_F_IS_SET(BUTT_3)) //3
	{ 
		LED_ON;	
	  Proc_Pow_ON();
	}
	
	if(BUTT_F_IS_SET(BUTT_4)) //4
	{
	  Proc_Pow_OFF();
	}
	
	if(BUTT_F_IS_SET(BUTT_5)) //5
	{
		LED_OFF;	
		BeepDelauyed(1,1,6);
	}
	
	if(BUTT_F_IS_SET(BUTT_6)) //stik
	{
		NVIC_GenerateSystemReset();	//LED_OFF;	
	}
}