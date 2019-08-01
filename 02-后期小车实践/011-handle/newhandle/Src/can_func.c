#include "can_func.h"

CAN_RX_DATA rx_data;
can_change_msg rx_msg;

void can_func_init()
{
    can_add_callback(999,can_show_lcd_func);
}

//can的中断接收处理函数
void can_test_func(CanRxMsgTypeDef* pRxMsg)
{
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    uprintf("%s\r\n",Data);  
}

void can_test_2_func(CanRxMsgTypeDef* pRxMsg)
{
    uint8_t Data[4];
    int i;
    for(i = 0; i < 4; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    uprintf("%d %d %d %d\r\n",(int)Data[0],(int)Data[1],(int)Data[2],(int)Data[3]);
}

void can_show_lcd_func(CanRxMsgTypeDef* pRxMsg)
{
    int i;
    for(i = 0; i < 8; i++)
    {
        rx_msg.ui8[i] = pRxMsg->Data[i];
    }
    
    if(pRxMsg->StdId == 999)
    {
         rx_data.posx = rx_msg.i16[0];
         rx_data.posy = rx_msg.i16[1];
         rx_data.angle = rx_msg.i16[2];
         rx_data.吗， = rx_msg.i16[3];
    }
}

void can_rx_show_lcd()
{
      Lcd_Show_Int(110,0,24,0,rx_data.posx);
      Lcd_Show_Int(110,24,24,0,rx_data.posy);
      Lcd_Show_Int(110,48,24,0,rx_data.angle);
      Lcd_Show_Int(110,72,24,0,rx_data.ccd);
}
