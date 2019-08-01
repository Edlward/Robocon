#ifndef __adc_func_H
#define __adc_func_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
  
#include "can.h"
#include "adc.h"
#include "lcd.h"
     
void adc_exe();
extern int adc_flag;
extern uint32_t ADC_Value[150];
extern int adc_power_off_flag;
     
     
#ifdef __cplusplus
}
#endif
#endif /*__ adc_func_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/