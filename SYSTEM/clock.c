#include "stm32f10x.h"
#include "clock.h"

/*
	！！！此文件必须独立设置为 O0 优化！！！
*/

volatile unsigned int SYS_ClockTick = 0;

void SysTick_Handler(void) {
	SYS_ClockTick++;
}

void clock_Init(void)
{
	SysTick_Config(SystemCoreClock/1000);   			// SysTick 1 msec interrupts
	NVIC_SetPriority (SysTick_IRQn, 0);					// 把时钟优先级设到最高
}


//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{	
	u32 n = nus<<4;										// 粗略认为16个循环是 1 us
	while(n) n--;
}

//延时nms
//nms为要延时的ms数.	
void delay_ms(u32 nms)
{	 		  	  
	u32 start = SYS_ClockTick;
	int time;
	while(1){
		time = SYS_ClockTick - start;
		if(time < 0) 	time += 0xFFFFFFFF;
		if(time >= nms)	return;
	}
} 
			
//获取当前时间戳
unsigned int get_ms(unsigned long *count)
{
	if(count)
		*count = SYS_ClockTick;
	return SYS_ClockTick;
}

