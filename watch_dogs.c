#include "watch_dogs.h"

void IWDG_Configuration(void)
{
RCC_LSICmd(ENABLE) ; //Open LSI
while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) ==RESET){} ; //It is steady to wait for LSI
  
  // Enable write access to IWDG_PR and IWDG_RLR registers /
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz /
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(RELOAD_WDG);
  // Reload IWDG counter /
  IWDG_ReloadCounter();  //1,5 sec
  // Enable IWDG (the LSI oscillator will be enabled by hardware) /
  IWDG_Enable();
}