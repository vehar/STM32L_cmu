#include "btns.h"

uint8_t btn_cnt[12], UpdateFlag[12], FlagCode[12];
uint32_t f_ButtPressed;

void Buttons_Init()
{
    GPIO_InitTypeDef GPIO_C, GPIO_B; 

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC , ENABLE);
	
	  GPIO_C.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_7;//5btns + encSw
    GPIO_C.GPIO_Mode = GPIO_Mode_IN;
    GPIO_C.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_C.GPIO_OType = GPIO_OType_PP;
    GPIO_C.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_DOWN
	  GPIO_Init(GPIOC, &GPIO_C);

	  GPIO_C.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_12;// STM_Out(speaker) + ON_GPS_stm
    GPIO_C.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_C.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_C.GPIO_OType = GPIO_OType_PP;
    GPIO_C.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_C);
	
	
		  GPIO_B.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_11;// on_akkum + red led 
    GPIO_B.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_B.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_B.GPIO_OType = GPIO_OType_PP;
    GPIO_B.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_B);
	
	 // GPIO_SetBits(GPIOC, GPIO_Pin_6);
	//  GPIO_SetBits(GPIOC, GPIO_Pin_7);
}

void Button_init_vs_irq (void){
	
 //  RCC->AHBENR  |= RCC_AHBENR_GPIOCEN; //enable gpioC
 //  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  /*!< System Configuration SYSCFG clock enable */
  /// GPIOA->CRH |=  GPIO_CRH_MODE0_1;
   //GPIOA->PUPDR |=  GPIO_PUPDR_PUPDR0_1; //BUTTON F in - pull down
   
  /// SYSCFG->EXTICR1 |= SYSCFG_EXTICR1_EXTI1 | SYSCFG_EXTICR1_EXTI1_PA;//?
   
 //  EXTI->EMR |= EXTI_EMR_MR0 ;// PIN 0     
 //  EXTI->IMR |= EXTI_IMR_MR0; //??? ??????? 0 
 //  EXTI->FTSR|= EXTI_FTSR_TR0 ; //???????????? - ?? ????????? ??????
   
   ///SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Connect EXTI line 0 to PA.0//?
  /// RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
   
 //  NVIC_EnableIRQ(EXTI0_IRQn); // Enable IRQn
//--------------------------------------------------------------------------------------------------------------
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN; //enable gpioC
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  /*!< System Configuration SYSCFG clock enable */
	
	
	//EXTI_DeInit();
	
 //Select Button pin as input source for EXTI Line 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);

   //Configure EXT1 Line 0 in interrupt mode trigged on Rising edge 
  EXTI_InitStructure.EXTI_Line |=  EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4 | EXTI_Line5;  // 
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

   //Enable and set EXTI.. Interrupt to the lowest priority 
  NVIC_InitStructure.NVIC_IRQChannel |=  EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn | EXTI3_IRQn | EXTI4_IRQn | EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 


//fall into empty? INT handler.......!!!!!?????

	/*NVIC_EnableIRQ (EXTI0_IRQn);
	NVIC_EnableIRQ (EXTI1_IRQn);
	NVIC_EnableIRQ (EXTI2_IRQn);
	NVIC_EnableIRQ (EXTI3_IRQn);
	NVIC_EnableIRQ (EXTI4_IRQn);
	NVIC_EnableIRQ (EXTI9_5_IRQn);	*/
	
}

void Butt_poll(void)
{
	if(BUTT_1) //1
	{
		BUTT_F_SET(F_BUTT_1);
	}
	if(BUTT_2) //2
	{
		BUTT_F_SET(F_BUTT_2);
	}
	if(BUTT_3) //3
	{
		BUTT_F_SET(F_BUTT_3);
	}
	if(BUTT_4) //4
	{
		BUTT_F_SET(F_BUTT_4);
	}
	if(BUTT_5) //5
	{
		BUTT_F_SET(F_BUTT_5);
	}
	if(BUTT_6) //stik
	{
		BUTT_F_SET(F_BUTT_6);
	}
}


void Btn_chk()
{
GPIO_ResetBits(GPIOC, GPIO_Pin_6);//disable
GPIO_SetBits(GPIOC, GPIO_Pin_7);	
		
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 1 )
 {	
	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 1)
	 {
	 }
 btn_cnt[0]++;
 UpdateFlag[0] = 1;	
 FlagCode[0] = KEY_F4;//0x51	 
 }
 
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1 )
 {	
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1)
	 {
	 }
 btn_cnt[1]++;
 UpdateFlag[1] = 1;	
 FlagCode[1] = 0x52;
 }
 
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1 )
 {	
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1)
	 {
	 }
 btn_cnt[2]++;	
 UpdateFlag[2] = 1;	
 FlagCode[2] = KEY_Menu;//0x53	 
 }
 
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == 1 )
 {	
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == 1)
	 {
	 }
 btn_cnt[3]++; 
 UpdateFlag[3] = 1;	
 FlagCode[3] = KEY_SingleCh;//0x54
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == 1)
	 {
	 }
 btn_cnt[4]++; 
 UpdateFlag[4] = 1;	
 FlagCode[4] = 0x55;
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == 1 )
 {	
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == 1)
	 {
	 }
 btn_cnt[5]++;
 UpdateFlag[5] = 1;	
 FlagCode[5] = 0x56;	 
 } 
 
 
 

GPIO_SetBits(GPIOC, GPIO_Pin_6);
GPIO_ResetBits(GPIOC, GPIO_Pin_7);//disable	 
 
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 1)
	 {
	 }
 btn_cnt[6]++;
 UpdateFlag[6] = 1;	
 FlagCode[6] = KEY_F3;//0x61
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1)
	 {
	 }
 btn_cnt[7]++;
 UpdateFlag[7] = 1;	
 FlagCode[7] = KEY_F2;//0x62
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 1)
	 {
	 }
 btn_cnt[8]++;
 UpdateFlag[8] = 1;	
 FlagCode[8] = KEY_F1;//0x63
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == 1)
	 {
	 }
 btn_cnt[9]++;
 UpdateFlag[9] = 1;	
 FlagCode[9] = KEY_DualCh;//0x64
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == 1)
	 {
	 }
 btn_cnt[10]++;
 UpdateFlag[10] = 1;	
 FlagCode[10] = KEY_Save;//0x65
 }
if( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == 1 )
 {
	 	 while( GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == 1)
	 {
	 }
 btn_cnt[11]++;
 UpdateFlag[11] = 1;	
 FlagCode[11] = KEY_Open;//0x66
 } 

GPIO_SetBits(GPIOC, GPIO_Pin_6);
GPIO_SetBits(GPIOC, GPIO_Pin_7); 
}

