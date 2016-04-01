#include "stm32l1xx_usart.h"
#include "stm32l1xx_gpio.h"

void uart_init();
uint8_t reciev_uart();
void sent_uart(uint8_t data);
void Usart1_Send_String(unsigned char* str);
void send_Uart_str( unsigned char *s);