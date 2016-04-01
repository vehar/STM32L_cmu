#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"




// INA219 memory registers
#define CONFIG_R		0x00	// configuration register
#define V_SHUNT_R		0x01	// differential shunt voltage
#define V_BUS_R			0x02	// bus voltage (wrt to system/chip GND)
#define P_BUS_R			0x03	// system power draw (= V_BUS * I_SHUNT)
#define I_SHUNT_R		0x04	// shunt current
#define CAL_R			0x05	// calibration register

#define INA_RESET		0xFFFF	// send to CONFIG_R to reset unit

#define CONFIG_DEFAULT		0x399F

// config. register bit labels
#define RST	15
#define BRNG	13
#define PG1	12
#define PG0	11
#define BADC4	10
#define BADC3	9
#define BADC2	8
#define BADC1	7
#define SADC4	6
#define SADC3	5
#define SADC2	4
#define SADC1	3
#define MODE3	2
#define MODE2	1
#define MODE1	0

// default values
#define D_I2C_ADDRESS	0x80 // (64)
#define D_RANGE			0
#define D_GAIN			3
#define D_SHUNT_ADC		3
#define D_BUS_ADC		3
#define D_MODE			7
#define D_SHUNT			0.1
#define D_V_BUS_MAX		32
#define D_V_SHUNT_MAX		0.32
#define D_I_MAX_EXPECTED	1 



void INA219_calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected);
void INA219_configure(uint8_t range, uint8_t gain, uint8_t bus_adc, uint8_t shunt_adc, uint8_t mode);
void INA219_reset();
int16_t INA219_shuntVoltageRaw();
float INA219_shuntVoltage();
int16_t INA219_busVoltageRaw();
float INA219_busVoltage();
float INA219_shuntCurrent();
uint16_t INA219_shuntCurrent_Raw();
float INA219_busPower();
uint16_t INA219_busVoltage_int();
uint16_t INA219_shuntCurrent_int();

void INA_Conf();