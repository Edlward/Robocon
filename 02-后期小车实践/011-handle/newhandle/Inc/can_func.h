#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
     
#include "lcd.h"
#include "can.h"
   
typedef union{
    uint8_t u8_form[4];
    int s32_form;
    float float_form;
}data_convert;

typedef struct CAN_RX_DATA{
    int now_speed;
    int max_speed;
    int posx;
    int posy;
    int angle;
    int laser_left;
    int laser_right;
    int press_left;
    int press_right;
    int ccd;
    
}CAN_RX_DATA;

typedef union{
        char ch[8];
        uint8_t ui8[8];
        int16_t i16[16];
        uint16_t ui16[4];
        int in[2];
        float fl[2];
        double df;
}can_change_msg;

void can_show_lcd_func(CanRxMsgTypeDef* pRxMsg);
void can_test_func(CanRxMsgTypeDef* pRxMsg);
void can_test_2_func(CanRxMsgTypeDef* pRxMsg);
void can_func_init();
void can_rx_show_lcd();
    
     
     
     
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/