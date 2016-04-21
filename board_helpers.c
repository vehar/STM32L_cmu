#include "board_helpers.h"


//My funktions_start
void HSI_on_16MHz (void)
{
   /*  internal HSI on 16MHz */  
  RCC->CR |= RCC_CR_HSION; 
  while(!(RCC_CR_HSION)); //wait until on
  RCC->CFGR |= RCC_CFGR_SW_HSI; //clock sourse is SYSCLK  HSI
  RCC->CR &= ~RCC_CR_MSION; //MSI turn off.   
}

void All_clk_On(void) //TODO: delete duplications!
{ 	
		//Enable the GPIOs clocks 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);      
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP |  RCC_APB1Periph_PWR , ENABLE);   //Enable comparator and PWR mngt clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG , ENABLE);//Enable ADC & SYSCFG clocks  

  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_SPI2EN ;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN;
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
 */ 
void RCC_Configuration(void)
{  
  RCC_HSICmd(ENABLE);//Enable HSI Clock  
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//!< Wait till HSI is ready  
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); //Set HSI as sys clock   
  RCC_MSIRangeConfig(RCC_MSIRange_6);//Set MSI clock range to ~4.194MHz*/  
  
  RCC_HSEConfig(RCC_HSE_OFF);  /*Disable HSE*/
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {    
    while(1); //Stay in infinite loop if HSE is not disabled*/
  }
}


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}


void Timer2_init_vs_irq(void)	
{
     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 Periph clock
     
    TIM2->PSC = SystemCoreClock / 1000 - 1; // 1000 tick/sec
    TIM2->ARR = 100;  // 1 Interrupt/0.1 sec
  
    TIM2->DIER |= TIM_DIER_UIE; // Enable tim2 interrupt
    TIM2->CR1 |= TIM_CR1_CEN;   // Start count
    
  //  NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable IRQ 
}


//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
#define CM3_SYSTICK_ENABLE				(1 << 0)
#define CM3_SYSTICK_CLKSOURCE			(1 << 2)
#define CM3_SYSTICK_COUNTFLAG			(1 << 16)

void stm32_delay_init(void)
{
	SysTick->CTRL = CM3_SYSTICK_CLKSOURCE;
	SysTick->VAL = 0;
}

void stm32_delay_delayus_do(uint32_t tick)
{
	uint32_t dly_tmp;
	
	stm32_delay_init();
	while (tick)
	{
		dly_tmp = (tick > ((1 << 24) - 1)) ? ((1 << 24) - 1) : tick;
		SysTick->LOAD = dly_tmp;
		SysTick->CTRL |= CM3_SYSTICK_ENABLE;
		while (!(SysTick->CTRL & CM3_SYSTICK_COUNTFLAG));
		stm32_delay_init();
		tick -= dly_tmp;
	}
}

void stm32_delay_delayus(uint16_t us) 
{
	//stm32_delay_delayus_do(us * (stm32_info.sys_freq_hz / (1000 * 1000)));
	stm32_delay_delayus_do(us * (RCC_Clocks.SYSCLK_Frequency / (1000 * 1000)));
}

void stm32_delay_delayms(uint16_t ms)
{
	//stm32_delay_delayus_do(ms * (stm32_info.sys_freq_hz / 1000));
	stm32_delay_delayus_do(ms * (RCC_Clocks.SYSCLK_Frequency / 1000)); 
}
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------