#include "breath_led.h"
#define  END  1

void breath_led_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOF时钟
    
    GPIO_Initure.Pin=GPIO_PIN_5; //PE5
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);	//PF9置1，默认初始化后灯灭
}

void delay_ns(u16 ns)  //延时;ns*14ns
{	   	
    for(u16 num=ns;num>0;num--)
    {
        __NOP();   
        
    }
    
} 


//0 码//
void  T0_init(void)
{
    //LED0=1;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
    delay_ns(5);
    //LED0=0;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
    delay_ns(20);
    
    
}
//1码//
void  T1_init()
{
    //LED0=1;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
    delay_ns(13);
    //LED0=0;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
    delay_ns(13);
    
}

//发送32位的 0码和1码
void send_color(u32 rgbw)
{
    u32 color;
    u8 r,g,b,w;	
    color=rgbw;
    r=color>>16;
    g=color>>24;
    b=color>>8;
    w=(u8)color;
    color=r<<24|g<<16|b<<8|w;
    // printf("color=%x\r\n",color);
    
    for(int i=31;i>=0;i--)
    {
        if(((1<<i)&color)>0)
        {
            T1_init();  //发送1码
            
        }	
        else 
        {
            T0_init(); //发送0码
            
        }
        
    }
    
    
}



//周期刷新码//
void Reset_init()
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
    //LED0=0;
    delay_us(80);
    
}

//将4个8位的rgbw合成一个32位数据
u32 read_rgbw(u8 r,u8 g,u8 b,u8 w)
{
    u32 rgbw;
    rgbw=r<<24|g<<16|b<<8|w;
    
    return  rgbw;
    
    
}

void  paoma_led(u32 co,u16 number_begin,u16 number_end,u8 direction)  //跑马灯，颜色，个数
{  
    uint32_t  color;
    if(direction==1)  //正着跑
    { 
        color=co;
        // 第一阶段 0——begin
        for(int m=0;m<=0;m++)
        { 
            if(m<number_begin)
            {
                send_color(0x00000000); //32wei颜色数据
                
            }
            if(m>=number_begin&m<=number_end)
            {
                send_color(color); //32wei颜色数据
            }  		 
            if(m>number_end&m<END)
            {
                send_color(0x00000000); //32wei颜色数据
            }
            
        }
	Reset_init();	
    }
}

   
