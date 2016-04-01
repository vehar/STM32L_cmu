
#include "i2c_hard_stm32l1.h"

GPIO_InitTypeDef gpio;
I2C_InitTypeDef i2c;
uint8_t  data1, data2, address, reg; 
uint16_t x;
uint8_t buffer1,buffer2;
volatile uint16_t i;

void delay()
{
    i = 1000;
    while ( i ) {
        i--;
		 
    }
}

void delay_long()
{
	volatile uint32_t del;
	for( del = 0; del<250000; del++);
}



void init_I2C1(void)
{
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
 
    
    i2c.I2C_ClockSpeed = 100000; 
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x15;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
 
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;			
    gpio.GPIO_Speed = GPIO_Speed_40MHz;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
    I2C_ITConfig(I2C1, I2C_AcknowledgedAddress_7bit, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
		delay();
}


//*******I2C ERROR REBOOTING*************

//**********I2C*****************
void i2c_write(uint8_t   address,uint8_t   reg)
{
//I2C_AcknowledgeConfig(I2C1, ENABLE);		
//Start
	
I2C1->CR1 |= I2C_CR1_START;
	// wait to ends Start
while (!(I2C1->SR1 & I2C_SR1_SB)){}

(void) I2C1->SR1;
// Address to write 
I2C1->DR = address;

//wait to ends sending of address	
while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
(void) I2C1->SR1;
(void) I2C1->SR2;

// Address of register
I2C1->DR = reg;
//wait to ends of address
while (!(I2C1->SR1 & I2C_SR1_BTF)){}
//Stop
I2C1->CR1 |= I2C_CR1_STOP;
delay();
}


void i2c_write_set(uint8_t address,uint8_t  reg,uint8_t  data1,uint8_t  data2)
{
I2C1->CR1 |= I2C_CR1_START;
	
while (!(I2C1->SR1 & I2C_SR1_SB)){	}

(void) I2C1->SR1;
I2C1->DR = address;
while (!(I2C1->SR1 & I2C_SR1_ADDR)){} 
(void) I2C1->SR1;
(void) I2C1->SR2;

I2C1->DR = reg;

while (!(I2C1->SR1 & I2C_SR1_BTF)){}

I2C1->DR = data1;

while (!(I2C1->SR1 & I2C_SR1_BTF)){}

I2C1->DR = data2;

while (!(I2C1->SR1 & I2C_SR1_BTF)){}

I2C1->CR1 |= I2C_CR1_STOP;

delay();
}

void i2c_write_set_1b(uint8_t address,uint8_t  reg,uint8_t  data1)
{
I2C1->CR1 |= I2C_CR1_START;
	
while (!(I2C1->SR1 & I2C_SR1_SB)){	}

(void) I2C1->SR1;
I2C1->DR = address;
while (!(I2C1->SR1 & I2C_SR1_ADDR)){} 
(void) I2C1->SR1;
(void) I2C1->SR2;

I2C1->DR = reg;

while (!(I2C1->SR1 & I2C_SR1_BTF)){}

I2C1->DR = data1;

while (!(I2C1->SR1 & I2C_SR1_BTF)){}

I2C1->CR1 |= I2C_CR1_STOP;
delay();
}



uint16_t i2c_read(uint8_t address,uint8_t  reg)
{	
I2C_AcknowledgeConfig(I2C1, ENABLE);	

//Start
I2C1->CR1 |= I2C_CR1_START;
	// wait to ends Start
while (!(I2C1->SR1 & I2C_SR1_SB)){	}

(void) I2C1->SR1;
// Address to read
I2C1->DR = address;
//wait to ends sending of address	
while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
 
(void) I2C1->SR1;
(void) I2C1->SR2;

// Address of register
I2C1->DR = reg;
//wait to ends of address
while (!(I2C1->SR1 & I2C_SR1_BTF)){}	

//************************************************************************

// repeat start
    I2C1->CR1 |= I2C_CR1_START;
 
    // wait to sb
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}
    (void) I2C1->SR1;
 
    // addr+1
    I2C1->DR = address+1;
 
    // wait
    while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
    (void) I2C1->SR1;
    (void) I2C1->SR2;
 
    // wait
    while (!(I2C1->SR1 & I2C_SR1_RXNE)){}
 
    // c???????? ???????? ????????
    buffer1 = I2C1->DR;		
		
	// wait
    while (!(I2C1->SR1 & I2C_SR1_RXNE)){}
		buffer2 = I2C1->DR;
		
		I2C_AcknowledgeConfig(I2C1, DISABLE);
I2C1->CR1 |= I2C_CR1_STOP;	
delay();
x = (((int)buffer1) <<8) | buffer2;

return x;
		
}

int i2c_read_1b(uint8_t address,uint8_t  reg)
{	
I2C_AcknowledgeConfig(I2C1, ENABLE);	
	
//Start
I2C1->CR1 |= I2C_CR1_START;
	// wait to ends Start
while (!(I2C1->SR1 & I2C_SR1_SB)){	}

(void) I2C1->SR1;
// Address to write 
I2C1->DR = address;
//wait to ends sending of address	
while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
 
(void) I2C1->SR1;
(void) I2C1->SR2;

// Address of register
I2C1->DR = reg;
//wait to ends of address
while (!(I2C1->SR1 & I2C_SR1_BTF)){}	

//************************************************************************

// repeat start
    I2C1->CR1 |= I2C_CR1_START;
 
    // wait to sb
    while (!(I2C1->SR1 & I2C_SR1_SB)){}
    (void) I2C1->SR1;
 
    // addr+1
    I2C1->DR = address+1;
 
    // wait
    while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
    (void) I2C1->SR1;
    (void) I2C1->SR2;
 
    // wait
    while (!(I2C1->SR1 & I2C_SR1_RXNE)){}
     
    buffer1 = I2C1->DR;		
		
	// wait
    while (!(I2C1->SR1 & I2C_SR1_RXNE)){}
		
		I2C_AcknowledgeConfig(I2C1, DISABLE);
I2C1->CR1 |= I2C_CR1_STOP;	
delay();


return buffer1;
		
}


