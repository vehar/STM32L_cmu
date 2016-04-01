#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "i2c_hard_stm32l1.h"
#include "spi.h"
#include "taskmgr.h"
#include "stm32l1xx_usart.h"
#include "uart.h"

void gps_on();
void gps_parser();
void gps_init();
void gps();
uint8_t time_return(uint8_t i);
uint8_t lat_return(uint8_t i);
uint8_t n_s_return();
uint8_t long_return(uint8_t i);
uint8_t e_w_return();