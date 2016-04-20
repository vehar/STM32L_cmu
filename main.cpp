#include "main.h"
	 
//#define WDG_ON disable watchdog for debug

#define MAX_CURRENT_LIM 6 // 6 means 600ma


#define F_WDG 			40000  // frq
#define PRESC_WDG 		32 // prs
#define T_WDG 	(float)1.5 // sec
#define RELOAD_WDG (F_WDG/PRESC_WDG)*T_WDG  // cal

void IWDG_Configuration(void);
void configureADC_Temp(void);
void Parse(uint8_t command);
void RCC_Configuration(void);
void BeepDelauyed(uint32_t time_on, uint32_t time_off, uint32_t iter);
//***********************************



/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;



/* Private define ------------------------------------------------------------*/

#define DEBUG_SWD_PIN   /* needs to be commented if SWD debug pins are not in use to reduce power consumption*/

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

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef DMA_InitStructure;

__IO uint16_t 	ADC_ConvertedValue, T_StartupTimeDelay;

uint32_t ADC_Result, INTemperature, refAVG, tempAVG, Address = 0;
int32_t temperature_C; 

uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];

char strDisp[20] ;

DisplayState_TypeDef CurrentlyDisplayed = Display_TemperatureDegC;

CALIB_TypeDef calibdata;    // field storing temp sensor calibration data 


volatile bool flag_ADCDMA_TransferComplete;
volatile bool flag_UserButton;

static volatile uint32_t TimingDelay;

__IO uint16_t   val_ref, val_25, val_110;

__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

*/
RCC_ClocksTypeDef RCC_Clocks;
/* Private function prototypes -----------------------------------------------*/
void  RCC_Configuration(void);
void  RTC_Configuration(void);
void  Init_GPIOs (void);
void  acquireTemperatureData(void);
void  configureADC_Temp(void);
void  configureDMA(void);
void  powerDownADC_Temper(void);
void  processTempData(void);
void  configureWakeup (void);
void  writeCalibData(CALIB_TypeDef* calibStruct);
/*
FunctionalState  testUserCalibData(void);
FunctionalState  testFactoryCalibData(void);*/
void insertionSort(uint16_t *numbers, uint32_t array_size);
uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples);
void clearUserButtonFlag(void);
/*******************************************************************************/


uint8_t  sizeofbuffer; 

float y;

float  result, r1, r2, r3, result1, h1, ina_current; 

uint16_t 	tmp,  
			command_type_rx, command_type_tx, 
			ss, ss2, ss0, adc1, adc2, coin=0;

uint8_t   cnt_last=0, Enc_cnt_actual=0,counter, size_of_data_handling, start_counter, t_res, size, status_buffer, tik, SPI_Work_Rx=0, SPI_Work_Tx=0, it,  data, needUpdate;

uint16_t XOR, control_counter, a1, a2, a3, a4;


//************FU*********************		
//***************************************************************
uint8_t Xor()
{
uint8_t Clock, Xor = 0;
uint8_t CNT = getSPI_TX(2);	

for(Clock=1; Clock <= CNT+2; Clock++)
	{	 	
		Xor^=getSPI_TX(Clock);
	}
return Xor;
}

void btn_handling()
{
uint8_t btn_data[12], clk_count[12], cnt_btn=0, clock=0,pck=0;

	for(clock = 0; clock<12; clock++)
	{
		if(UpdateFlag[clock] == 1)
		{
		cnt_btn++;
			
		btn_data[cnt_btn] = FlagCode[clock];
		clk_count[cnt_btn] = btn_cnt[clock];
			
		UpdateFlag[clock] = 0;
		FlagCode[clock] = 0;
		btn_cnt[clock] = 0;
			
		}
	}	
	
setSPI_TX(START_POS,START_BYTE);
setSPI_TX(COMAND_POS, KBD_DATA);//0x50
setSPI_TX(PACKED_SIZE_POS, (cnt_btn*2));
pck=3;

	for(clock = 1; clock<=cnt_btn; clock++)
	{
		setSPI_TX(pck, clk_count[clock] );
		pck++;
		setSPI_TX( pck, btn_data[clock] );
		pck++;
		clk_count[clock] = 0;
		btn_data[clock] = 0;
	}

if(cnt_btn==0)
{
	setSPI_TX(pck-1, 0x01);

	setSPI_TX(pck, 0x00);
	pck++;
	setSPI_TX(pck, (0x50^0x00^0x01) );
	pck++;
	setSPI_TX(pck, STOP_1_BYTE);
	pck++;
	setSPI_TX(pck, STOP_2_BYTE);
	pck++;	
	StartSPI_Tx_Only(pck);	
}
else{
	//pck++;
	setSPI_TX(pck, Xor());
	pck++;
	setSPI_TX(pck, STOP_1_BYTE);
	pck++;
	setSPI_TX(pck, STOP_2_BYTE);
	pck++;
	StartSPI_Tx_Only(pck);	
}
coin=0;	

}



void DM_send(uint8_t command_type_tx, uint8_t size_of_data)
{
	
uint8_t d1=0, clk, XOR=0;	
	
SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);	
	
setSPI_TX(START_POS,START_BYTE); //Start Byte 0xAA
setSPI_TX(COMAND_POS, command_type_tx);
setSPI_TX(PACKED_SIZE_POS, size_of_data);	

	
	for( clk=1; clk <= ( size_of_data+2 ); clk++ )
	{
	  d1 = getSPI_TX(clk);	
	  XOR=XOR^d1;			
	}		

setSPI_TX((size_of_data+3), XOR); //XOR
setSPI_TX((size_of_data+4), STOP_1_BYTE); //1st Stop byte 0x55
setSPI_TX((size_of_data+5), STOP_2_BYTE);	//2nd Stop byte	0xff
	
StartSPI_Tx_Only((size_of_data+6));		
	
delay(); //delay for DM if something wrong	
	
SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);	
}




void WordPack(uint16_t word)
{
	uint8_t H = 0;
	uint8_t L = 0;
	    H = (word>>8)&0xff;
        L = word&0xff;
        setSPI_TX(3,H);		
        setSPI_TX(4,L);	
}


void rx_handling()
{
	
uint8_t Command_type;	

if( getSPI_RX(7) != 0) //HV - why? need to extend
{
SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
	coin = 0;
	needUpdate = 0;
	Command_type = getSPI_RX(1);
	Parse(Command_type);	
SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	
}
setSPI_RX(7, 0x00);
setSPI_RX(0, 0x00);
}

void DM_reciev()
{
	NVIC_DisableIRQ(SPI2_IRQn);
		
	StartSPI_Rx_Only(10);
		
	rx_handling();
		
	NVIC_EnableIRQ(SPI2_IRQn);
}

void hvl()
{
GPIO_InitTypeDef gpio;
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_40MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    //gpio.GPIO_PuPd = GPIO_PuPd_DOWN;

	 GPIO_Init(GPIOA,&gpio);
	
	// GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void Current_chk()
{
ina_current=INA219_shuntCurrent();
ina_current=ina_current*10;
if(	ina_current > MAX_CURRENT_LIM)
{}
else{}
}

void Voltage_chk() //TODO: implement!
{
 INA219_busVoltage();
}

void Enc_chk()
{
Enc_cnt_actual = enc_GetRelativeMove();	
	
	if(	Enc_cnt_actual != cnt_last)
	{
		Parse(ENCODER_DATA);//	0x80	
		Enc_cnt_actual = cnt_last;		
	}

	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)== 1)
	{
		Parse(0x81); //NOT IMPLEMENTED YET!	
	}
}



void tmg_dm(uint8_t pid) //https://docs.google.com/document/d/1nXo-8PuYDCrjen-9jSjmz0lAr2kqzddpsU6W7EBHXk8/edit#
{ 
	Current_chk();	
	Voltage_chk();	
	//Temperature_chk();
	//Axel_chk();
	Btn_chk();		
	Enc_chk();
	
	
	
	rx_handling();	
		
	if(getSPI_RX(0) == 0x00)
	{
		coin = 0;	
	}
	//task_kill(pid);
}

/*
void tmg_recive(uint8_t pid)
{ 
	NVIC_DisableIRQ(SPI2_IRQn);
	
DM_reciev();	
task_kill(pid);
	
	NVIC_EnableIRQ(SPI2_IRQn);
}
*/



//My funktions_start
void HSI_on_16MHz (void){
   /*  internal HSI on 16MHz */  
  RCC->CR |= RCC_CR_HSION; 
  while(!(RCC_CR_HSION)); //wait until on
  RCC->CFGR |= RCC_CFGR_SW_HSI; //clock sourse is SYSCLK  HSI
  RCC->CR &= ~RCC_CR_MSION; //MSI turn off.   
}

void All_clk_On(void){ //
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_SPI2EN ;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN;
}


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void Timer2_init_vs_irq(void){
     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 Periph clock
     
    TIM2->PSC = SystemCoreClock / 1000 - 1; // 1000 tick/sec
    TIM2->ARR = 100;  // 1 Interrupt/0.1 sec
  
    TIM2->DIER |= TIM_DIER_UIE; // Enable tim2 interrupt
    TIM2->CR1 |= TIM_CR1_CEN;   // Start count
    
  //  NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable IRQ 
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



//RCC_ClocksTypeDef* RCC_Clocks;


void init_mcu_fu()
{
  //SystemInit();
	 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_md.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 
	
  //  HSI_on_16MHz(); 
	
    All_clk_On(); 
 //Timer2_init_vs_irq(); //ovr_irq
	
	
	uart_init();		
	init_timer();	
	init_I2C1();	
	//Spi_hw_init(); //init in Comm obj
	
	Buttons_Init(); //! todo 
	Button_init_vs_irq();
	
	configureADC_Temp();//HV
	adc_init();
	//hvl();
		
	delay();
	//gps_init();		
	//INA_Conf();	
	
#ifdef WDG_ON
	//IWDG_Configuration();	
#endif	
}




/*************** M *******************
**************** A *******************
**************** I *******************
**************** N ********************/

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


uint32_t f_ButtPressed = 0;
bool f_PowActive = false;
bool f_SystemOk = false;


void Proc_Pow_ON(void)
{
 if(f_PowActive == false)
	{			
	f_PowActive = true;
		LED_ON;
	MAIN_RWR_ON;
	BeepDelauyed(10,10,5);	
	}
}

void Proc_Pow_OFF(void)
{
 if(f_PowActive == true)
	{			
	f_PowActive = false;
	BeepDelauyed(1,5,5);
	MAIN_RWR_OFF;	
		LED_OFF;
	}
}


bool Axel_temperature_check(void)
{
	
}

#define STM_TEMPERATURE_MIN  0 //C
#define STM_TEMPERATURE_MAX  90 //C

bool STM_temperature_check(void)//TODO need debug!
{
 uint32_t  data = 0;
float	Current_Temperature = 0;
	
 ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1 ,ADC_SampleTime_384Cycles);
 ADC_SoftwareStartConv(ADC1);
	
	//stm32_delay_delayms(1);
	
 data  = ADC1->DR;
	
	Current_Temperature = data;
                        Current_Temperature  = Current_Temperature * 3.3 / 4095;  // 3.3V / (2^12-1) 
                        Current_Temperature -= 0.760;       // 25°C  25°C, V = 0.760V)
                        Current_Temperature /= 0.0025;       //  25°C (??? STM32F407 2.5mV / 1°C)
                       // Current_Temperature += 25.0;        // 
						Current_Temperature -=273;
	
	
	if((Current_Temperature > STM_TEMPERATURE_MIN)&&(Current_Temperature <= STM_TEMPERATURE_MAX)) 
		return true;
	else 
		return false;
}




#define AKK_VOLTAGE_MIN  63 //6.3v
#define AKK_VOLTAGE_MAX  74 //7.4v

bool Batt_voltage_check(void)
{
	uint32_t data = INA219_busVoltageRaw();
	if((data > AKK_VOLTAGE_MIN)&&(data <= AKK_VOLTAGE_MAX)) 
		return true;
	else 
		return false;
}

#define AKK_CURRENT_MIN  100 //100mA
#define AKK_CURRENT_MAX  500 //500mA
bool Batt_current_check(void)
{
	uint32_t data = INA219_shuntCurrent_Raw();
	if((data > AKK_CURRENT_MIN)&&(data <= AKK_CURRENT_MAX)) 
		return true;
	else 
		return false;
}

uint32_t Prelaunch_verification(void)
{
	uint32_t result = 0;
	
	RCC_GetClocksFreq(&RCC_Clocks);//!!!!!!! must be 16MHz
	
	if(RCC_Clocks.SYSCLK_Frequency == 16000000) 	{result ++;}
	if(STM_temperature_check()) 	{result ++;}
	if(Axel_temperature_check()) 	{result ++;}
//	if(Batt_voltage_check()) 		{result ++;}
	
	
	if(result == 3){ f_SystemOk = true;} //if all is ok - set f_SystemOk flag & ready to start
	else f_SystemOk = false;
	
	return result;
}

uint32_t AdcData = 0;
int main(void) 
{
	//Communication Comm;		//Communication protocol vs STM

//////////////////////////////////////////////////////////////////////////////////
 ///////////////////////On-StartTestSection_start////////////////////////////////// 
  RCC_Configuration();   //Configure Clocks for Application need 
  
  //PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);    //Set internal voltage regulator to 1.8V 
  //while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;   //Wait Until the Voltage Regulator is ready 
 ///////////////////////On-StartTestSection_end//////////////////////////////////// 
 //////////////////////////////////////////////////////////////////////////////////
	
	init_mcu_fu();

/*	__enable_irq();			
	NVIC_EnableIRQ(SPI2_IRQn);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
*/		

	Prelaunch_verification();
	
	if(f_SystemOk) //system & device ok	
	{
		Proc_Pow_ON();
	}
	
while(1)
{
	
	STM_temperature_check();//DEBUG!
	
     AdcData = ADC1->JDR1;//	 stm_adc_5v_DATA: //0x03
	//AdcData = AdcData * 3.3 / 4095;
	 AdcData = ADC1->JDR2;// stm_adc_140v_DATA: //0x04
	//AdcData = AdcData * 3.3 / 4095;
	
	if(BUTT_1) //1
	{
		LED_ON;	
		//f_ButtPressed = 1;
		BeepDelauyed(1,1,2);
	}
	
	if(BUTT_2) //2
	{
		LED_ON;	
		STM_temperature_check();
	}
	
	if(BUTT_3) //3
	{
		//LED_ON;	
	  Proc_Pow_ON();

	}
	
	if(BUTT_4) //4
	{
	  Proc_Pow_OFF();
	}
	
	if(BUTT_5) //5
	{
		LED_OFF;	
		BeepDelauyed(1,1,6);
	}
	
	if(BUTT_6) //stik
	{
		NVIC_GenerateSystemReset();	//LED_OFF;	
	}
	
	/*if(f_ButtPressed) //10-10 pisk
	{
		LED_ON;	
		BeepDelauyed(100,100);	
	}*/
	
}
	
	task_add(*IWDG_ReloadCounter);		
	task_add(*tmg_dm);
		
	taskmgr();	
}

//************************************

void BeepDelauyed(uint32_t time_on, uint32_t time_off, uint32_t iter) 
{
	for(uint32_t i = 0; i<iter; i++)
	{
		SPEAKER_ON;
	stm32_delay_delayus(time_on);
		SPEAKER_OFF;
	stm32_delay_delayus(time_off);	
	}
}

void SPI2_IRQHandler()//TODO: add transmition!
{	
	uint8_t num = 0;
	//Rx================================================================================
	 if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
  {
    //Receive SPI2 data
    //data = SPI_I2S_ReceiveData(SPI2);//just an idea here as how receive needs to be done

	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);//???
	setSPI_RX(coin, SPI_I2S_ReceiveData(SPI2) );	
	coin++;	
	
    //clear interrupt bit
    SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
  }
  
	//Tx================================================================================
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) == SET)
  {
    //Send SPI2 data
    //SPI_I2S_SendData(SPI2, data);//just an idea here as how send needs to be done
	  SPI_I2S_SendData(SPI2, SPI_Buffer_Tx[num] ); num++; //TODO: fix this inc!!!!!!
    //clear interrupt bit
    SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_TXE);
  }

  
  //================================================================
	
  //SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
  
    /* SPI Error interrupt--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_OVR) == SET) //TODO: test!!!!!!
  {
   // SPI_ReceiveData8(SPI1);
		//STM_EVAL_LEDOn(LED9);
    SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_OVR);
  }
  
}
//************************************
 void HardFault_Handler(void)
{	
	NVIC_GenerateSystemReset();	
}

 void EXTI0_IRQHandler(void)
{
  /* Disable general interrupts */
  //disableInterrupts();
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line0);
  //enableInterrupts();
	__enable_irq();
}

 void EXTI1_IRQHandler(void)
{
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line1);
	__enable_irq();
}

 void EXTI2_IRQHandler(void)
{
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line2);
	__enable_irq();
}
 void EXTI3_IRQHandler(void)
{
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line3);
	__enable_irq();
}
 void EXTI4_IRQHandler(void)
{
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line4);
	__enable_irq();
}
 void EXTI9_5_IRQHandler(void)
{
	__disable_irq();
 
    
  EXTI_ClearITPendingBit(EXTI_Line5);
	__enable_irq();
}


void IWDG_Configuration(void)
{
RCC_LSICmd(ENABLE) ; //Open LSI
while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) ==RESET){} ; //It is steady to wait for LSI
  
  // Enable write access to IWDG_PR and IWDG_RLR registers /
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz /
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(RELOAD_WDG);
  // Reload IWDG counter /
  IWDG_ReloadCounter();  //1,5 sec
  // Enable IWDG (the LSI oscillator will be enabled by hardware) /
  IWDG_Enable();
}


void Parse(uint8_t command)
{	
uint8_t m = 0;
uint8_t l = 0;
uint8_t i = 0;
uint16_t data = 0, tmp  =0;

switch (command)
{
    case 0x00:
     break;
     
    case raw_volt_DATA: //0x01
        data=INA219_busVoltageRaw();
     break;
    
    case raw_curr_DATA: //0x02
    	data=INA219_shuntCurrent_Raw();
     break;
 
    case stm_adc_5v_DATA: //0x03
        data = ADC1->JDR1;	
     break; 
    
    case stm_adc_100v_DATA: //0x04
        data = ADC1->JDR2;	
     break;   
    
    case KBD_DATA://0x50 //****** BUTTON ******************
        btn_handling();	
     break;
    
    case ENCODER_DATA: //0x80
        setSPI_TX(3, Enc_cnt_actual);		
        DM_send(ENCODER_DATA,1);//DM_send(0x80,1);	
     break;    
    
    case TEMPERATURE://0xB3
        data = get_ds3231_temp();	
	 break;
	
	 case STM_TEMPERATURE://0xB4 //TODO need debug!
		ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1 ,ADC_SampleTime_384Cycles);
		ADC_SoftwareStartConv(ADC1);
        data  = ADC1->DR;		
	 break;
    
    case BTN_CNT_RST: //0xC1
        btn_cnt[ getSPI_RX(3)] = 0;
     break;

    case 0xC2:
        Enc_cnt_actual = 0	;
     break;

    case 0xff:
        setSPI_TX(0, 0xFF);	
        setSPI_TX(1, 0x55);	
        setSPI_TX(2, 0x03);	
        setSPI_TX(3, 0x02);	
        setSPI_TX(4, 0x01);	
        setSPI_TX(5, 0xFF);	
        setSPI_TX(6, 0xAA);		
        StartSPI_Tx_Only(7);
     break;
 
    default: //TODO: test
		data = 0;
		command = ERROR;
		
    break;
}
	//Send prepared data
	WordPack(data);
	DM_send(command,2);


if(command == 0x70)
{
gps();
	for(i=0;i<6;i++) 
	{
		setSPI_TX( i+3 , time_return(i) );
	}	
DM_send(0x70,6);	
}

if(command == 0x71)
{
gps();
	for(i=0;i<6;i++)
	{
		setSPI_TX( i+3 , lat_return(i) );
	}	
DM_send(0x71,6);
}

if(command == 0x72)
{
gps();
	for(i=0;i<6;i++)
	{
		setSPI_TX( i+3 , long_return(i) );
	}	
DM_send(0x72,6);
}

if(command == 0x73)
{
	gps();	
	setSPI_TX(3, e_w_return());	
	DM_send(0x73,1);	
}

if(command == 0x74)
{
	gps();	
	setSPI_TX(3, n_s_return());	
	DM_send(0x74,1);	
}

if(command == 0xA0)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);	
}	

if(command == 0xA1)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4);	
}	
if(command == 0xB0)
{
	setSPI_TX(3, get_sec());	
	DM_send(0xB0,1);	
}	
if(command == 0xB1)
{
	setSPI_TX(3, get_min());	
	DM_send(0xB1,1);	
}	

if(command == 0xB2)
{
	setSPI_TX(3, get_hour());	
	DM_send(0xB2,1);	
}	


if(command == 0xB4)
{
	set_time((uint8_t)getSPI_RX(3), (uint8_t)getSPI_RX(4), (uint8_t)getSPI_RX(5));
}	
	
if(command == 0xB5)
    {
    uint8_t clk1, clk2, buf_raw, time_raw[6];	
        
    uint8_t time_buf[3];	
        
    gps();


    for(clk2=0; clk2<6; clk2++)
        {
            
         buf_raw = time_return(clk2);
            
       for(clk1=0; clk1<10; clk1++)
         {
            if( buf_raw == (0x30+clk1) )
            {
                time_raw[clk2] = clk1;
            }           
         }      
      }
              
    m = time_raw[0];
    m = m*10;
    l = time_raw[1];	
    time_buf[0] = m + l;

    m = time_raw[2];
    m = m*10;
    l = time_raw[3];	
    time_buf[1] = m + l;
        
    m = time_raw[4];
    m = m*10;
    l = time_raw[5];	
    time_buf[2] = m + l;
             
    set_time(time_buf[0], time_buf[1], time_buf[2]);
    }
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
  
    /* Enable  comparator clock LCD and PWR mngt */
  RCC_APB1PeriphClockCmd(/*RCC_APB1Periph_LCD |*/ RCC_APB1Periph_PWR, ENABLE);
    
  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);

  	//Enable the GPIOs clocks 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);      
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP |  RCC_APB1Periph_PWR , ENABLE);   //Enable comparator and PWR mngt clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG , ENABLE);//Enable ADC & SYSCFG clocks  
}




//BeepDelauyed(1,1);	    //ultra
//BeepDelauyed(10,10);	    //pisk loud
//BeepDelauyed(100,100);	//norm
//BeepDelauyed(100,200);	//hruk	
//BeepDelauyed(200,200);	//hruk
