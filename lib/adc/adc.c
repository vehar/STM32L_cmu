#include "adc.h"

void adc_init()
{
GPIO_InitTypeDef    GPIO_InitStructure;
ADC_InitTypeDef ADC_InitStructure;	
	
RCC_HSICmd(ENABLE); 	
while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET); 
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);	

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
GPIO_Init(GPIOA, &GPIO_InitStructure); 	
	
ADC_InitStructure.ADC_ScanConvMode = ENABLE; //We will convert single channel only
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//we will convert many times  
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //right 12-bit data alignment in ADC data register  
ADC_Init(ADC1, &ADC_InitStructure);//load structure values to control and status registers

ADC_InjectedSequencerLengthConfig(ADC1, 2);

ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_192Cycles);	
ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_192Cycles);

ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_None);

ADC_AutoInjectedConvCmd(ADC1, ENABLE);

ADC_Cmd(ADC1, ENABLE);

//TODO: delay();

ADC_SoftwareStartInjectedConv(ADC1);  

//TODO: delay();
}


/////////////////////////////////////////////////
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;


void acquireTemperatureData(void)
{
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET); 

  /* re-initialize DMA -- is it needed ?*/
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* Enable DMA channel 1 Transmit complete interrupt*/
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Disable DMA mode for ADC1 */ 
  ADC_DMACmd(ADC1, DISABLE);

   /* Enable DMA mode for ADC1 */  
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Clear global flag for DMA transfert complete */
 // clearADCDMA_TransferComplete(); 
  
  /* Start ADC conversion */
  ADC_SoftwareStartConv(ADC1);
}

void configureADC_Temp(void)
{
  uint32_t ch_index;

  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* Enable the internal connection of Temperature sensor and with the ADC channels*/
  ADC_TempSensorVrefintCmd(ENABLE); 
  
  /* Wait until ADC + Temp sensor start */
 uint32_t T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);

  /* Setup ADC common init struct */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  
  /* Initialise the ADC1 by using its init structure */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	          // Set conversion resolution to 12bit
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          // Enable Scan mode (single conversion for each channel of the group)
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  // Disable Continuous conversion
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; // Disable external conversion trigger
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  // Set conversion data alignement to right
  ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE;             // Set conversion data alignement to ADC_CONV_BUFF_SIZE
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular Temperature sensor channel16 and internal reference channel17 configuration */ 

    for (ch_index = 1; ch_index <= MAX_TEMP_CHNL; ch_index++){
      ADC_RegularChannelConfig(ADC1, ADC_Channel_16, ch_index, 
                               ADC_SampleTime_384Cycles);
    }

  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 17, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 18, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 19, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 20, ADC_SampleTime_384Cycles);
}

void powerDownADC_Temper(void)
{
  /* Disable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);  
  /* Disable ADC1 */
  ADC_Cmd(ADC1, DISABLE);

  /* Disable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);  
  /* Disable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
}