//board_helpers.h
#ifndef __BOARD_HELPERS_H
#define __BOARD_HELPERS_H

#include "board.h"


void HSI_on_16MHz (void);
void All_clk_On(void); //TODO: delete duplications!
void RCC_Configuration(void);
void NVIC_GenerateSystemReset(void);
void Timer2_init_vs_irq(void);


void stm32_delay_init(void);
void stm32_delay_delayus_do(uint32_t tick);
void stm32_delay_delayus(uint16_t us);
void stm32_delay_delayms(uint16_t ms);

#endif //__BOARD_HELPERS_H