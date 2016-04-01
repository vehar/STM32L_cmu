#include "timer.h"

void init_timer(void)
{ 
  GPIO_InitTypeDef gpio_cfg;
	TIM_TimeBaseInitTypeDef timer_base;
	
  GPIO_StructInit(&gpio_cfg);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
	gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpio_cfg.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &gpio_cfg);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  gpio_cfg.GPIO_Mode = GPIO_Mode_IN;
	gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpio_cfg.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &gpio_cfg);
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
timer_base.TIM_Prescaler = 0;
timer_base.TIM_ClockDivision = TIM_CKD_DIV1;
timer_base.TIM_Period = 255;
timer_base.TIM_CounterMode = TIM_CounterMode_Up;  
TIM_TimeBaseInit(TIM3, &timer_base);
TIM_ARRPreloadConfig(TIM3, ENABLE);

TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

TIM2->CNT = 0;
TIM_Cmd(TIM3, ENABLE); 
}

int16_t enc_GetRelativeMove(void)
{
    return (int8_t)TIM3->CNT;
}