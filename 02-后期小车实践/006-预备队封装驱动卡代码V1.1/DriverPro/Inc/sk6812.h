#ifndef __sk6812_H
#define __sk6812_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
     
typedef struct RGB
{	
  uint8_t red;
  uint8_t green;
  uint8_t blue;
}color;

   
extern color White;
extern color Black;
extern color Red;
extern color Green;
extern color Blue;
extern color Cyan;
extern color Yellow;
extern color Magenta;
extern color Orange;

extern color sk6812_color;
extern color Rainbow_Array[7];

extern uint16_t sk_duty_data[400];
void send_color(color RGB);
color Rainbow_color(int i , int full);
     
#ifdef __cplusplus
}
#endif
#endif /*__sk6812_H */