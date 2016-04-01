#include "stm32l1xx.h"
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"

#define MAX_RUNNING_TASKS 10

uint8_t task_add(void *task);
uint8_t task_kill(uint8_t pid);
uint8_t get_runningtasks(void);
void taskmgr(void);


