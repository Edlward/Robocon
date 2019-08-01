#include "sk6812.h"
#include "tim.h"
#include "usart.h"
#include "math.h"

color White = {255,255,255};
color Black ={0,0,0};
color Red ={255,0,0};
color Green= {0,255,0};
color Blue= {0,0,255};
color Cyan= {0,255,255};
color Yellow ={255,255,0};
color Magenta = {255,0,255};
color Orange = {255,152,0};
color Violet = {150,0,255};
color Rainbow_Array[7] = {{255,0,0}, {255,152,0}, {255,255,0}, {0,255,0}, {0,255,255}, {0,0,255}, {150,0,255}};

uint16_t sk_duty_data[400]={0};
color sk6812_color= {0,0,0};
#define T0duty 22
#define T1duty 43


//发送32位的 0码和1码
void send_color(color RGB)
{
  uint32_t Color=0;
  Color=RGB.green<<16|RGB.red<<8|RGB.blue;
  
  for(int i=0;i<=23;i++)
  {
    if(1&(Color>>(23-i)))
    {
      sk_duty_data[i] = T1duty;  //发送1码
      
    }	
    else 
    {
      sk_duty_data[i] = T0duty;//发送0码
      
    }
    
  }
  
  
}
color Rainbow_color(int i , int full){
  uint8_t r,g,b;
  if(i<full/3){
    r=255;
    g=(uint8_t)ceil(255*3*i/full);
    b=0;
  }else if(i<full/2){
    r=(uint8_t)ceil(750-i*(250*6/full));
    g=255;
    b=0;
  }else if(i<full*2/3){
    r=0;
    g=255;
    b=(uint8_t)ceil(i*(250*6/full)-750);
  }else if(i<full*5/6){
    r=0;
    g=(uint8_t)ceil(1250-i*(250*6/full));
    b=255;
  }else{
    r=(uint8_t)ceil(150*i*(6/full)-750);
    g=0;
    b=255;
  }
  color retcolor = {r,g,b};
  return retcolor;
}



