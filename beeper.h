//beeper
#ifndef __BEEPER_H
#define __BEEPER_H

#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "board.h"
#include "board_helpers.h"

void BeepDelauyed(uint32_t time_on, uint32_t time_off, uint32_t iter); 


#endif //__BEEPER_H 