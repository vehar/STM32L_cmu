
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_iwdg.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_exti.h"

#include "stm32l1xx_dbgmcu.h"

#include "stm32l1xx_it.h"

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


#define BUTT_1 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)== 0)
#define BUTT_2 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)== 0)	
#define BUTT_3 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)== 0)	
#define BUTT_4 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)== 0)	
#define BUTT_5 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)== 0)	
#define BUTT_6 (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)== 0)	//stik
	
#define LED_ON	GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define LED_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_11)	

#define SPEAKER_ON	GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define SPEAKER_OFF	GPIO_ResetBits(GPIOC, GPIO_Pin_6)

#define MAIN_RWR_ON	    GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define MAIN_RWR_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_0)


extern RCC_ClocksTypeDef RCC_Clocks;









