//#pragma once

#ifdef __cplusplus
 extern "C" {
#endif 
	 
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_iwdg.h"


void init_timer(void);
int16_t enc_GetRelativeMove(void);

#ifdef __cplusplus
}
#endif
