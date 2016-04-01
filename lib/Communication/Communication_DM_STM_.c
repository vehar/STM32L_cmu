#include "Communication_DM_STM.h"

uint8_t Xor()
{
uint8_t Clock, Xor = 0;
uint8_t CNT = getSPI_TX(2);	

for(Clock=1; Clock <= CNT+2; Clock++)
	{	 	
		Xor^=getSPI_TX(Clock);
	}
return Xor;
}