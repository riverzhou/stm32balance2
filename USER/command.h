#ifndef _COMMAND_H_
#define _COMMAND_H_


extern void command_proc(unsigned char data);
extern void command_init(void);

#define CMDLEN	16
struct usart_buff_t{
	unsigned int 	index;
	unsigned char buff[CMDLEN];
};
extern struct usart_buff_t* CMD;

#endif
