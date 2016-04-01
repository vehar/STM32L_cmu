#include "stm32l1xx.h"
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "i2c_hard_stm32l1.h"

#ifdef __cplusplus
 extern "C" {
#endif 
	 
	 
#define CS_On() GPIO_ResetBits(GPIOB,GPIO_Pin_12);
#define CS_Off() GPIO_SetBits(GPIOB,GPIO_Pin_12);

#define size_of_buffer_tx 256
#define size_of_buffer_rx 8

	 
extern uint8_t SPI_Buffer_Tx[size_of_buffer_tx];
extern uint8_t SPI_Buffer_Rx[size_of_buffer_rx];
	 
extern void Spi_hw_init(void);

extern void StartSPI_Rx_Only(uint16_t number);
extern void StartSPI_Tx_Only(uint16_t number);
	 
#define LPVOID 	void*	 
extern void SPI_exchange(LPVOID lpInBuf, LPVOID lpOutBuf, int size);
	 
extern void setSPI_TX(uint16_t Adr, uint8_t Data);
extern uint16_t getSPI_RX(uint16_t Adr);
extern uint16_t getSPI_TX(uint16_t Adr);


extern void spi_send_byte(uint8_t data) ;
extern void setSPI_RX(uint16_t Adr, uint8_t Data);


#ifdef __cplusplus
}
#endif
