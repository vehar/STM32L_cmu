#include "ina219.h"
#include "i2c_hard_stm32l1.h"

float r_shunt,power_lsb,current_lsb; 
uint8_t m,l,config;


//********INA219 - CURRENT MONITOR***************
void INA219_calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected)
{
  uint16_t cal;
  float i_max_possible, min_lsb, max_lsb, swap;

  r_shunt = shunt_val;

  i_max_possible = v_shunt_max / r_shunt;
	
	
  min_lsb = i_max_expected / 32767;
  max_lsb = i_max_expected / 4096;

  current_lsb = (uint16_t)(min_lsb * 100000000) + 100;
  current_lsb /= 100000000;
	

  swap = (0.04096)/(current_lsb*r_shunt);
  cal = (uint16_t)swap;
  power_lsb = current_lsb * 22.7;

 m|=cal>>8;
 l|=cal;
		
i2c_write_set(D_I2C_ADDRESS, CAL_R, m, l);
	
}





void INA219_configure(uint8_t range, uint8_t gain, uint8_t bus_adc, uint8_t shunt_adc, uint8_t mode)
{
  uint8_t config = 0;
  config |= (range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);
  m|=config>>8;
  l|=config;
  i2c_write_set(D_I2C_ADDRESS, CONFIG_R, m, l);	
}

// resets the INA219
void INA219_reset()
{
 	i2c_write_set(D_I2C_ADDRESS, CONFIG_R, 0xff, 0xff);	
  delay();
}




// returns the raw binary value of the shunt voltage
int16_t INA219_shuntVoltageRaw()
{	 
	 return i2c_read(0x80,V_SHUNT_R);
}

// returns the shunt voltage in volts.
float INA219_shuntVoltage()
{
  float temp;
	temp = i2c_read(0x80,V_SHUNT_R);
  return (temp / 100000);
}

// returns raw bus voltage binary value
int16_t INA219_busVoltageRaw()
{
	
	return i2c_read(0x80,V_BUS_R);
  
}

// returns the bus voltage in volts
float INA219_busVoltage()
{
  int16_t temp;
	temp = i2c_read(0x80,V_BUS_R);
  temp >>= 3;
  return (temp * 0.004);
}

// returns the shunt current in amps
float INA219_shuntCurrent()
{
  return (i2c_read(0x80,I_SHUNT_R) * current_lsb);
}

// returns the shunt current in amps
uint16_t INA219_shuntCurrent_Raw()
{
  return i2c_read(0x80,I_SHUNT_R);
}


// returns the bus power in watts
float INA219_busPower()
{
  return (i2c_read(0x80,P_BUS_R) * power_lsb);
}
//*********************************************
uint16_t INA219_busVoltage_int()
{
  int16_t temp;
	temp = i2c_read(0x80,V_BUS_R);
  temp >>= 3;
  return ((temp * 0.0045)*100);
}

// returns the shunt current in amps
uint16_t INA219_shuntCurrent_int()
{
  return ((i2c_read(0x80,I_SHUNT_R) * current_lsb)*100);
}




void INA_Conf()
{
	INA219_reset();
	INA219_configure(D_RANGE,D_GAIN,D_BUS_ADC,D_SHUNT_ADC,D_MODE);
	INA219_calibrate(D_SHUNT,D_V_SHUNT_MAX,D_V_BUS_MAX,D_I_MAX_EXPECTED);
	INA219_configure(D_RANGE,D_GAIN,D_BUS_ADC,D_SHUNT_ADC,D_MODE);
	INA219_calibrate(D_SHUNT,D_V_SHUNT_MAX,D_V_BUS_MAX,D_I_MAX_EXPECTED);		
}
