#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"




 void delay();
void I2C1_init(void);

void i2c_write(uint8_t address,uint8_t  reg);
void i2c_write_set(uint8_t  address,uint8_t  reg,uint8_t  data1,uint8_t  data2);
void i2c_write_set_1b(uint8_t address,uint8_t  reg,uint8_t  data1);
uint16_t i2c_read(uint8_t address,uint8_t  reg);
int i2c_read_1b(uint8_t address,uint8_t  reg);
