#pragma once

#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "spi.h"
#include "taskmgr.h"
#include "board.h"
#include "board_helpers.h"

//Key codes
#define KEY_DualCh   0x64
#define KEY_Save	 0x65
#define KEY_Open	 0x66
//#define KEY_A_B		 0x62
#define KEY_F1		 0x63
#define KEY_F2		 0x62
#define KEY_F3		 0x61
#define KEY_F4		 0x51
#define KEY_Menu	 0x53
#define KEY_SingleCh 0x54
#define KEY_Return	 0x85
#define KEY_Help	 0x86



#define BUTT_1 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)== 0)
#define BUTT_2 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)== 0)	
#define BUTT_3 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)== 0)	
#define BUTT_4 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)== 0)	
#define BUTT_5 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)== 0)	
#define BUTT_6 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)== 0)	//stik
	

#define F_BUTT_0 0
#define F_BUTT_1 1
#define F_BUTT_2 2
#define F_BUTT_3 3
#define F_BUTT_4 4
#define F_BUTT_5 5
#define F_BUTT_6 6
#define F_BUTT_7 7
	
	
#define BUTT_F_SET(button) 		(f_ButtPressed|=(1<<button))
#define BUTT_F_RESET(button) 	(f_ButtPressed&=(~(1<<button)))
#define BUTT_F_IS_SET(button)	((f_ButtPressed&(1<<button)) != 0)
#define BUTT_F_IS_RESET(button) ((f_ButtPressed&(1<<button)) == 0)



extern uint32_t f_ButtPressed;
extern uint8_t btn_cnt[12], UpdateFlag[12], FlagCode[12];

void Buttons_Init();
void Button_init_vs_irq (void);
void Butt_poll(void);
void Btn_chk();
