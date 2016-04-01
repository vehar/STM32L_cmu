#include "ds3231.h"
#include "i2c_hard_stm32l1.h"

uint8_t temp_msb, temp_lsb;
uint16_t result_temp;

//***************DS3231 - RTC *******************
uint8_t  get_sec()
{
	return i2c_read_1b(DS3132_ADDRESS,RTC_SECONDS);
}

uint8_t  get_min()
{		
return i2c_read_1b(DS3132_ADDRESS,RTC_MINUTES);		
}

uint8_t  get_hour()
{	
return i2c_read_1b(DS3132_ADDRESS,RTC_HOURS);	
}


void set_time(uint8_t hour, uint8_t min, uint8_t sec)
{		
hour|=0x40;	
i2c_write_set_1b(DS3132_ADDRESS, RTC_SECONDS, sec);
i2c_write_set_1b(DS3132_ADDRESS, RTC_MINUTES,min);
i2c_write_set_1b(DS3132_ADDRESS, RTC_HOURS,hour);	
}
	
void Start_init()
{		
i2c_write_set_1b(DS3132_ADDRESS, RTC_SECONDS, 0x00);	
i2c_write_set_1b(DS3132_ADDRESS, RTC_MINUTES, 0x00);
i2c_write_set_1b(DS3132_ADDRESS, RTC_HOURS,0x40);	
}
	
uint16_t get_temp()
{

temp_msb = i2c_read_1b(DS3132_ADDRESS, TEMP_MSB);	
	
temp_lsb = i2c_read_1b(DS3132_ADDRESS, TEMP_LSB);		
	

result_temp = temp_msb << 2 ;
	
temp_lsb = temp_lsb >> 6 ;	
	
result_temp +=	temp_lsb  ;
	
result_temp &= 0x7f;	
	
return result_temp;
	
}


uint16_t get_STM_temp()
{
uint16_t result = 0;

	
return result;
}