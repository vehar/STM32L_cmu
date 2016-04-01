#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"


 
#define DS3132_ADDRESS 0xd0
//DS3232 Register Addresses
#define RTC_SECONDS 0x00
#define RTC_MINUTES 0x01
#define RTC_HOURS 0x02
#define RTC_DAY 0x03
#define RTC_DATE 0x04
#define RTC_MONTH 0x05
#define RTC_YEAR 0x06
#define ALM1_SECONDS 0x07
#define ALM1_MINUTES 0x08
#define ALM1_HOURS 0x09
#define ALM1_DAYDATE 0x0A
#define ALM2_MINUTES 0x0B
#define ALM2_HOURS 0x0C
#define ALM2_DAYDATE 0x0D
#define RTC_CONTROL 0x0E
#define RTC_STATUS 0x0F
#define RTC_AGING 0x10
#define TEMP_MSB 0x11
#define TEMP_LSB 0x12
#define SRAM_START_ADDR 0x14    //first SRAM address
#define SRAM_SIZE 236           //number of bytes of SRAM

uint8_t get_sec();
uint8_t  get_min();
uint8_t  get_hour();
void set_time(uint8_t hour, uint8_t min, uint8_t sec);
void Start_init();
uint16_t get_temp();