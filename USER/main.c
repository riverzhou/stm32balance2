#include <stdio.h>
#include "usart1.h"

int main(void)
{
	
	usart1_Init(72,115200);
	
	
	while(1)
		printf("Hello World ! \r\n");
	//测试串口打印
	
}

