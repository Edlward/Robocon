#ifndef __breath_H
#define __breath_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "lcd.h"
     
#define u32 uint32_t
#define u8 uint8_t
#define u16 uint16_t
     
void breath_led_Init(void);  
void  paoma_led(u32 co,u16 number_begin,u16 number_end,u8 direction);  //跑马灯，颜色，个数
     
     
     
     
     
     
     
     
     
     
     
#ifdef __cplusplus
}
#endif
#endif /*__breath_H */
