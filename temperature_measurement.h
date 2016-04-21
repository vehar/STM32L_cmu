//temperature_measurement.h

#ifndef __TEMPERATURE_MEASUREMENT_H
#define __TEMPERATURE_MEASUREMENT_H

#include "stm32l1xx.h"
#include <stdio.h>

#define FACTORY_CALIB_BASE        ((uint32_t)0x1FF80078)    /*!< Calibration Data Bytes base address */
#define FACTORY_CALIB_DATA        ((CALIB_TypeDef *) FACTORY_CALIB_BASE)
#define USER_CALIB_BASE           ((uint32_t)0x08080000)    /*!< USER Calibration Data Bytes base address */
#define USER_CALIB_DATA           ((CALIB_TypeDef *) USER_CALIB_BASE)
#define TEST_CALIB_DIFF           (int32_t) 50  /* difference of hot-cold calib
                                               data to be considered as valid */ 

#define HOT_CAL_TEMP 110
#define COLD_CAL_TEMP  25

#define DEFAULT_HOT_VAL 0x362
#define DEFAULT_COLD_VAL 0x2A8

#define MAX_TEMP_CHNL 16

#define TS_110

#define AVG_SLOPE 	1620
#define V90		597000
#define V_REF		3300000

#define ADC_CONV_BUFF_SIZE 20


typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;

/*
__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;

uint32_t ADC_Result, INTemperature, refAVG, tempAVG, Address = 0;
int32_t temperature_C; 

uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];

__IO uint16_t   val_ref, val_25, val_110;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;*/




void  acquireTemperatureData(void);
void  configureADC_Temp(void);
void  configureDMA(void);
void  powerDownADC_Temper(void);
void  processTempData(void);
void  configureWakeup (void);
void  writeCalibData(CALIB_TypeDef* calibStruct);

FunctionalState  testUserCalibData(void);
FunctionalState  testFactoryCalibData(void);

#endif //__TEMPERATURE_MEASUREMENT_H
