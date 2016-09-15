#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "command.h"

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/

struct usart_buff_t usart_buff;
struct usart_buff_t *CMD = &usart_buff;

void command_proc(unsigned char data)
{	
	CMD->index += 1;
	CMD->index %= CMDLEN;
	CMD->buff[CMD->index] = data;
	//printf("\r\ncommand_proc %d \r\n", data);
} 

void command_init(void)
{
	memset(CMD, 0, sizeof(struct usart_buff_t));
	CMD->index = CMDLEN-1;
}

