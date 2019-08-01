#include "can_func.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "utils.h"
#include "motor.h"
#include <stdlib.h>

void can_handle_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  }
  switch(can_RX_data.in[0])
  {
  case 0:
      if(can_RX_data.in[1] > 100 ||can_RX_data.in[1] < 0) break;
      Control_Mode=PWM_Mode;
      if(Motor==BRUSH)
          Brush_PWM_Control(can_RX_data.in[1]);
      else
          Set_Motor_Duty(can_RX_data.in[1]);
      break;
  case 1:
      Control_Mode=Speed_Mode;
      Target_Speed=can_RX_data.in[1];
      break;
  case 2:
      Control_Mode=Position_Speed_Mode;
      Target_Position=can_RX_data.in[1];
      break;
  }
}