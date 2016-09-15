#include <string.h>
#include "stm32f10x.h"
#include "usart.h"
#include "command.h"
#include "clock.h"


/**************************************************************************
函数功能：串口1接收中断
入口参数：无
返回  值：无
**************************************************************************/

void USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){			//接收到数据
		command_proc(USART_ReceiveData(USART1));
	}  
} 

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/

void usart1_Init(void) 
{
	int bound = 115200;
	
	//GPIO端口设置
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);				//使能UGPIOA 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);				//使能USART1 时钟
	
	//USART1_TX
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9; 								//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;				 		//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);											//初始化GPIOA.9

	//USART1_RX	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin  	= GPIO_Pin_10;								//PA.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;			//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);											//初始化GPIOA.10  

  //USART1 初始化设置
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate 		= bound;										//串口波特率
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;			//字长为8位数据格式
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;					//一个停止位
	USART_InitStructure.USART_Parity 			= USART_Parity_No;					//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART1, &USART_InitStructure);     										//初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);										//开启串口接受中断
	USART_Cmd(USART1, ENABLE);																				//使能串口1

	//Usart1 NVIC 配置
	memset(&NVIC_InitStructure,0,sizeof(NVIC_InitStructure));
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);														//根据指定的参数初始化VIC寄存器

}

/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int usart1_PutChar (int ch) {
  while (!(USART1->SR & 0x0080));
  USART1->DR = (ch & 0xFF);
  return (ch);
}

/*----------------------------------------------------------------------------
  Read character from Serial Port
 *----------------------------------------------------------------------------*/
int usart1_GetChar (void) {
  if (USART1->SR & 0x0020)
    return (USART1->DR);
  return (-1);
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){			//接收到数据
		command_proc(USART_ReceiveData(USART3));
	}  
} 

void usart3_Init(void)
{
	int bound = 9600;
	
	//GPIO端口设置
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);			//使能UGPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);			//使能USART3时钟
	
	//USART3_TX
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_10; 								//PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;						//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
	//USART3_RX
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11;								//PB11
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN_FLOATING;			//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  //USART 初始化设置
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate 		= bound;										//串口波特率
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;			//字长为8位数据格式
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;					//一个停止位
	USART_InitStructure.USART_Parity 			= USART_Parity_No;					//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART3, &USART_InitStructure);     										//初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);										//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    										//使能串口3 
	
	//Usart3 NVIC 配置
	memset(&NVIC_InitStructure,0,sizeof(NVIC_InitStructure));
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	//抢占优先级4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);														//根据指定的参数初始化VIC寄存器
}

/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int usart3_PutChar (int ch) {
  while (!(USART3->SR & 0x0080));
  USART3->DR = (ch & 0xFF);
  return (ch);
}

/*----------------------------------------------------------------------------
  Read character from Serial Port
 *----------------------------------------------------------------------------*/
int usart3_GetChar (void) {
  if (USART3->SR & 0x0020)
    return (USART3->DR);
  return (-1);
}
