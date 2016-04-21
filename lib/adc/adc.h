//#pragma once
#ifndef __ADC_H
#define __ADC_H


#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_iwdg.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_adc.h"
#include "misc.h"

//#include "main.h"

#define HOT_CAL_TEMP 			110
#define COLD_CAL_TEMP 			 25

#define DEFAULT_HOT_VAL 		0x362
#define DEFAULT_COLD_VAL 		0x2A8

#define MAX_TEMP_CHNL 			16

#define TS_110

#define AVG_SLOPE 				1620
#define V90						597000
#define V_REF					3300000

#define ADC_CONV_BUFF_SIZE 		20

void adc_init();

void powerDownADC_Temper(void);
void configureADC_Temp(void);
void acquireTemperatureData(void);


extern ADC_InitTypeDef ADC_InitStructure;
extern ADC_CommonInitTypeDef ADC_CommonInitStructure;

extern void setADCDMA_TransferComplete(void);
extern void clearADCDMA_TransferComplete(void);

extern void clearADCDMA_TransferComplete(void);
/*extern void setADCDMA_TransferComplete(void);*/
#endif //__ADC_H