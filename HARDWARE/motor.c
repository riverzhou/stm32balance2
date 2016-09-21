#include <stdlib.h>
#include "motor.h"
#include "ioi2c.h"

#define MAX_ANGEL			50				// 50度
#define MIN_VOLTAGE		1110			// 11.1V

void Motor_Init(void)
{		 		
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 				//使能PB端口时钟
	
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	//端口配置
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     				//50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      				//根据设定参数初始化GPIOB 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOA, ENABLE);		// 使能GPIO外设时钟使能

  //设置该引脚为复用输出功能,输出TIM1 CH1 CH4的PWM脉冲波形
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8|GPIO_Pin_11; 			//TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;  						//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period 				= 7199; 							// ((1+TIM_Prescaler )/72M)*(1+TIM_Period)	 
	TIM_TimeBaseStructure.TIM_Prescaler 		= 0; 									// 10 KHZ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 							//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse 			= 0;                      //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity 	= TIM_OCPolarity_High;    //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  										//根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  										//根据TIM_OCInitStruct中指定的参数初始化外设TIMx

  TIM_CtrlPWMOutputs(TIM1,ENABLE);															//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  						//CH1预装载使能	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  						//CH4预装载使能	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); 													//使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM1, ENABLE);  																			//使能TIM1
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
#define PWMA   TIM1->CCR1  //PA8
#define PWMB   TIM1->CCR4  //PA11

#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN1   PBout(13)
#define BIN2   PBout(12)

void Set_Pwm(int moto1,int moto2)
{
	int Amplitude = 7000;    			//===PWM满幅是7200 限制在7000
	int Moto1=moto1, Moto2=moto2;
	
  if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;		
	
	if(Moto1 < 0)			
		AIN2=1, AIN1=0;
	else 	          
		AIN2=0, AIN1=1;
	if(Moto2 < 0)	
		BIN1=0, BIN2=1;
	else        
		BIN1=1, BIN2=0;
	PWMA = abs(Moto1);
	PWMB = abs(Moto2);	
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
int Turn_Off(float angle, int voltage)
{
	if(angle < -MAX_ANGEL || angle > MAX_ANGEL || voltage < MIN_VOLTAGE)
	{	                                                 //===倾角大于40度关闭电机，或者电压低于11.1V
		AIN1=0;                                          //===可自行增加主板温度过高时关闭电机
		AIN2=0;
		BIN1=0;
		BIN2=0;
		return 1;
	}
  return 0;			
}

