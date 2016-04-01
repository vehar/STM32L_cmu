#include "uart.h"


void uart_init()
{
GPIO_InitTypeDef GPIO_InitStructure;  
USART_InitTypeDef USART_InitStructure;

	
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
 

GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); 


// PA9 -> TX UART.
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure); 
 
//PA10  -> RX UART. 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_Init(GPIOA, &GPIO_InitStructure);



RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
        
 
  /* USART2 configured as follow:
  - BaudRate = 9600 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART1->BRR = (uint8_t)(8000000/38400);
  
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
 
  USART1->CR1 |= USART_CR1_UE;	
	
  USART_Cmd(USART1, ENABLE);

}
uint8_t reciev_uart()
{ 
uint8_t data;
while(!(USART1->SR & USART_SR_RXNE)){}; 
data = USART1->DR;
return data;
}
void sent_uart(uint8_t data)
{
while(!(USART1->SR & USART_SR_TC)){}; 
		
USART_SendData(USART1,data);
	
}
void Usart1_Send_String(unsigned char* str)
{
  uint8_t i=0;
  while(str[i])
  {
    sent_uart(str[i]);
    i++;
  }
	
}

