//#pragma once
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "spi.h"
#include "taskmgr.h"


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

extern uint8_t btn_cnt[12], UpdateFlag[12], FlagCode[12];

void Buttons_Init();
void Btn_chk();
