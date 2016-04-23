#include "beeper.h"

void BeepDelauyed(uint32_t time_on, uint32_t time_off, uint32_t iter) 
{
	for(uint32_t i = 0; i<iter; i++)
	{
		SPEAKER_ON;
	stm32_delay_delayus(time_on);
		SPEAKER_OFF;
	stm32_delay_delayus(time_off);	
	}
}  