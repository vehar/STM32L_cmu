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
#include "board.h"

#define MAX_TEMP_CHNL 			16
#define ADC_CONV_BUFF_SIZE 		20


//Limit value defines
//=====================================================
//Main power
#define AKK_VOLTAGE_MIN_YELOW  65 //6.5v
#define AKK_VOLTAGE_MAX_YELOW  72 //7.2v

#define AKK_VOLTAGE_MIN_RED  63 //6.3v
#define AKK_VOLTAGE_MAX_RED  74 //7.4v
//------------------------------------------------------

#define AKK_CURRENT_MIN_YELOW  110 //110mA
#define AKK_CURRENT_MAX_YELOW  400 //400mA

#define AKK_CURRENT_MIN_RED  100 //100mA
#define AKK_CURRENT_MAX_RED  500 //500mA
//------------------------------------------------------


//Internal voltage converters values
#define U_140_V_MAX 141
#define U_140_V_MIN 139

#define U_5000_mV_MAX 5500
#define U_5000_mV_MIN 4900

#define U_3300_mV_MAX 3400
#define U_3300_mV_MIN 3100

#define U_1800_mV_MAX 1900
#define U_1800_mV_MIN 1700

//=====================================================

extern uint32_t f_device_FAULT;

#define F_FAULT_140V 	 1
#define F_FAULT_5000mV 2
#define F_FAULT_3300mV 3	
#define F_FAULT_1800mV 4	

#define FAULT_F_SET(fault_f) 		(f_device_FAULT|=(1<<fault_f))
#define FAULT_F_RESET(fault_f) 	(f_device_FAULT&=(~(1<<fault_f)))


extern uint32_t device_140_v;
extern uint32_t device_5000_mv;
extern uint32_t device_3300_mv;
extern uint32_t device_1800_mv;


void adc_init();

void powerDownADC_Temper(void);
void configureADC_Temp(void);
void acquireTemperatureData(void);


extern ADC_InitTypeDef ADC_InitStructure;
extern ADC_CommonInitTypeDef ADC_CommonInitStructure;

extern void setADCDMA_TransferComplete(void);
extern void clearADCDMA_TransferComplete(void);

extern void clearADCDMA_TransferComplete(void);

void devise_voltages_chk(void);
/*extern void setADCDMA_TransferComplete(void);*/
#endif //__ADC_H