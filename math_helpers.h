//math_helpers.h
#ifndef __MATH_HELPERS_H
#define __MATH_HELPERS_H

#include "stm32l1xx.h"

void insertionSort(uint16_t *numbers, uint32_t array_size);
uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples);


#endif //__MATH_HELPERS_H