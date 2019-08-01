#include "motor.h"
#include "usart.h"
#include <math.h>
#include "robomaster.h"

#define MAXCURRENT 2
int init_pos_flag = 0;
double MAXSPEED = 10000;
double MAXSPEED_neg = 10000;
Brush_Phase Brush_chl_AB={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}, .Other_Phase = C};
Brush_Phase Brush_chl_BC={.Brush_Phase_pos={B,C},.Brush_Phase_neg={C,B}, .Other_Phase = A};
Brush_Phase Brush_chl_AC={.Brush_Phase_pos={A,C},.Brush_Phase_neg={C,A}, .Other_Phase = B};
Brush_Phase Brush_chl={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}, .Other_Phase = C};

void Brush_PWM_Control(float pwm){
  Duty=pwm;
  if(pwm>=0){
    Set_Phase_High_Speed(Brush_chl.Brush_Phase_pos.High,pwm);
    Set_Phase_High_Speed(Brush_chl.Brush_Phase_pos.Low,LOW_CLOSE);
    Set_Phase_High_Speed(Brush_chl.Other_Phase,LOW_CLOSE);
  }
  else{
    Set_Phase_High_Speed(Brush_chl.Brush_Phase_neg.High,-pwm);
    Set_Phase_High_Speed(Brush_chl.Brush_Phase_neg.Low,LOW_CLOSE);
    Set_Phase_High_Speed(Brush_chl.Other_Phase,LOW_CLOSE);
  }
}

void PWM_Control(float duty){
  if(Motor==BRUSH)
    Brush_PWM_Control(duty);
  else if(Motor == ROBOMASTER)
  {
      robomaster_set_current2(canmtrid,(int16_t)duty);
  }
  else
    Set_Motor_Duty(duty);
}

void Control()
{
  float current_out=0;
  float speed_out=0;
  float position_out=0;
  //RoboconMaster_Control();
  //gpio_dectect();
  if(Control_Mode&Position_Mode)
  {
    position_out=PID_Release(&Position_PID,Target_Position,Now_Position);
    if(Control_Mode&Speed_Mode)
    {
      Target_Speed=position_out;
      Limit(Target_Speed,MAXSPEED);
      speed_out=PID_Release(&Speed_PID,Target_Speed,Now_Speed);
      if(Control_Mode&Current_Mode)
      {//Position_Speed_Current 0x07
        Target_Current=speed_out;
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        Limit(current_out,95);
        PWM_Control(current_out);
      }
      else//Position_Speed 0x06
      {   Limit(speed_out,95);
      PWM_Control(speed_out);
      }
    }
    else
    {
      if(Control_Mode&Current_Mode)
      {//Position_Current 0x05
        Target_Current=position_out;
        Limit(Target_Current,MAXCURRENT);
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//Position 0x04
      {
        Limit(position_out,95);
        PWM_Control(position_out);  
      }
    }
    
  }
  else{
    if(Control_Mode&Speed_Mode)
    {
      Limit(Target_Speed,MAXSPEED);
      speed_out=PID_Release(&Speed_PID,Target_Speed,Now_Speed);
      if(Control_Mode&Current_Mode){//Speed_Current 0x03
        Target_Current=speed_out;
        Limit(Target_Current,MAXCURRENT);
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//Speed 0x02
        PWM_Control(speed_out);
    }
    else
    {
      if(Control_Mode&Current_Mode){//Current 0x01
        Limit(Target_Current,MAXCURRENT);
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//PWM 0x00
        ;
    }
  }
}

void gpio_dectect()
{
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET
       && fabs(Target_Position-Now_Position) < 50)
    {
        Control_Mode=Speed_Mode;
        Target_Speed = 0;
    }
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET
       && init_pos_flag == 1)
    {
        Control_Mode=Speed_Mode;
        Target_Speed = 0;
        Now_Position = 0;
        Target_Position = 0;
        //MAXSPEED = flash_data[13];
        init_pos_flag = 0;
    }
}
