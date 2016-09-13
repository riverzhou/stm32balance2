#include "stm32f10x.h"
#include "led.h"

//#define LED PAout(12) // PA12

/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12;	        //端口配置
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;     //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
}

#if 0
/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	static int 	temp = 0;
	if(0 == time) 
		LED = 0;
	else		
		if(++temp == time)	
			LED = ~LED, temp = 0;
}
#endif

