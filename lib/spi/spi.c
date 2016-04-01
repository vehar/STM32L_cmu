#include "spi.h"

#ifdef __cplusplus
 extern "C" {
#endif 
	 
	 
void Spi_init();

uint8_t SPI_work=0;

uint8_t SPI_Buffer_Tx[size_of_buffer_tx];
uint8_t SPI_Buffer_Rx[size_of_buffer_rx];

	
//Number - num of bytes 



void StartSPI_Rx_Only(uint16_t number)
{	
uint8_t num;	

	for(num = 0; num < number; num++ )
	 {		  
			SPI_Buffer_Rx[num] = SPI_I2S_ReceiveData(SPI1); 		
	 }		
}
			
void StartSPI_Tx_Only(uint16_t number)
{
uint8_t num;	

	for(num = 0; num < number; num++ )
	{	
		spi_send_byte( SPI_Buffer_Tx[num] );	
	}	
spi_send_byte(0x00);	
}



void setSPI_RX(uint16_t Adr, uint8_t Data)
{
	
	if( Adr < size_of_buffer_tx)
	  {
		SPI_Buffer_Rx[Adr]=Data;
	  }
	
}


void setSPI_TX(uint16_t Adr, uint8_t Data)
{
	
	if( Adr < size_of_buffer_tx)
	  {
		SPI_Buffer_Tx[Adr]=Data;
	  }
	
}


uint16_t getSPI_RX(uint16_t Adr)
{
	
		 return  SPI_Buffer_Rx[Adr];
	
}

uint16_t getSPI_TX(uint16_t Adr)
{
	
		 return  SPI_Buffer_Tx[Adr];
	
}

void Spi_hw_init()
{ 
    GPIO_InitTypeDef gpio;
	  SPI_InitTypeDef spi2;
	
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	
 //*******************************************************   
    GPIO_StructInit(&gpio);
 
    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_40MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
 //*******************************************************   	
    GPIO_Init(GPIOB,&gpio);
 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
    
    SPI_I2S_DeInit(SPI2);
   
    SPI_StructInit(&spi2);
 
    spi2.SPI_DataSize = SPI_DataSize_8b;
		spi2.SPI_CPOL = SPI_CPOL_Low;
		spi2.SPI_CPHA = SPI_CPHA_1Edge;
		spi2.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
		spi2.SPI_FirstBit = SPI_FirstBit_MSB; 
		spi2.SPI_Mode = SPI_Mode_Slave;		
		spi2.SPI_NSS = SPI_NSS_Soft;
				
    SPI_Init(SPI2, &spi2);
		
    SPI_Cmd(SPI2,ENABLE);		
		
  	delay();
}
//*****************************************************************************
void spi_send_byte(uint8_t data) 
{		
//while (!(SPI1->SR & SPI_SR_TXE));		
SPI_I2S_SendData(SPI2, data);		
while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}		
}  
	
void SPI_exchange(LPVOID lpInBuf, LPVOID lpOutBuf, int size) //TODO: ADD DMA
{
	uint8_t send_data, temp;
	
  SPI2->DR = send_data;//slave
  SPI1->DR = 0x0F; //m
  while(!(SPI1->SR & SPI_SR_RXNE));
  temp = SPI1->DR;
}
#ifdef __cplusplus
}
#endif









