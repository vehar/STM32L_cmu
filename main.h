#pragma once

#include "stm32l1xx.h"
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_iwdg.h"

#ifdef __cplusplus
 extern "C" {
#endif 
	 
#include "i2c_hard_stm32l1.h"
#include "ina219.h"
#include "ds3231.h"
#include "spi.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h"
#include "misc.h"
#include "stm32l1xx_adc.h"
#include "taskmgr.h"
#include "stm32l1xx_usart.h"
#include "uart.h"
#include "gps.h"

#include "btns.h"  
#include "adc.h"
#include "timer.h"

#ifdef __cplusplus
}
#endif

#include "Communication_DM_STM.h"

//protocol defines

#define START_BYTE  0xAA
#define STOP_1_BYTE 0x55
#define STOP_2_BYTE 0xFF

#define START_POS			0
#define COMAND_POS			1
#define PACKED_SIZE_POS		2
#define DATA_1_POS			3
#define DATA_2_POS			4



//COMANDS
#define ERROR		0xFF

#define raw_volt_DATA		0x01
#define raw_curr_DATA		0x02
#define stm_adc_5v_DATA		0x03
#define stm_adc_100v_DATA	0x04

#define KBD_DATA		0x50
#define MOUSE_DATA		0x06
#define RAW_DATA		0x07
#define ENCODER_DATA    0x80
#define BTN_CNT_RST     0xC1 
#define TEMPERATURE     0xB3 
#define STM_TEMPERATURE 0xB4 



