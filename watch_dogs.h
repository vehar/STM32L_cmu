//watch_dogs
#ifndef __WATCH_DOGS_H
#define __WATCH_DOGS_H 

#include "stm32l1xx.h"
#include "stm32l1xx_iwdg.h"

#define F_WDG 			40000  // frq
#define PRESC_WDG 		32 // prs
#define T_WDG 	(float)1.5 // sec
#define RELOAD_WDG (F_WDG/PRESC_WDG)*T_WDG  // cal

void IWDG_Configuration(void);


#endif //__WATCH_DOGS_H