//#pragma once
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l1xx.h"
#include <stdio.h>
//#include "board.h"//In main.c
//---------------------------------------------
/*
ADC_InitTypeDef ADC_InitStructure;
extern ADC_CommonInitTypeDef ADC_CommonInitStructure;
extern DMA_InitTypeDef DMA_InitStructure;*/

void TimingDelay_Decrement(void);


void Delay(__IO uint32_t nTime);

#endif /* __MAIN_H */