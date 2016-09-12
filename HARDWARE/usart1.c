/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low Level Serial Routines
 * Note(s): possible defines select the used communication interface:
 *            __DBG_ITM   - ITM SWO interface
 *                        - USART2 interface  (default)
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2014 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "usart1.h"

/*----------------------------------------------------------------------------
 Define  USART
 *----------------------------------------------------------------------------*/
#define USARTx  USART1

/*----------------------------------------------------------------------------
 Define  Baudrate setting (BRR) for USART
 *----------------------------------------------------------------------------*/
#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
 
void usart1_Init(u32 pclk2, u32 bound) {
	RCC->APB2ENR |= 1<<2;    		//使能PORTA口时钟  
	RCC->APB2ENR |= 1<<14;   		//使能串口时钟 
	
	GPIOA->CRH &= 0XFFFFF00F; 	//IO状态设置
	GPIOA->CRH |= 0X000008B0; 	//IO状态设置
		  
	RCC->APB2RSTR |= 1<<14;   	//复位串口1
	RCC->APB2RSTR &= ~(1<<14);	//停止复位	   	   
	
 	USART1->BRR = __USART_BRR(pclk2*1000000, bound); // 波特率设置	 
	USART1->CR1 |= 0X200C;  		//1位停止,无校验位.
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int SER_PutChar (int ch) {
  while (!(USARTx->SR & 0x0080));
  USARTx->DR = (ch & 0xFF);
  return (ch);
}

/*----------------------------------------------------------------------------
  Read character from Serial Port
 *----------------------------------------------------------------------------*/
int SER_GetChar (void) {
  if (USARTx->SR & 0x0020)
    return (USARTx->DR);
  return (-1);
}

//------------------------------------------------------------------------------

void SER_Put(unsigned char data)
{
	USARTx->DR = data;
	while((USARTx->SR&0x40)==0);	
}

int SER_Get(unsigned char *data)
{
	if (USARTx->SR & 0x0020){
		*data = (unsigned char)USARTx->DR;
		return 0;
	}
  return (-1);
}

