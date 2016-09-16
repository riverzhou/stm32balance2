#ifndef _COMMAND_H_
#define _COMMAND_H_

extern void command_proc(unsigned char data);
extern void command_init(void);

struct cmd_buff_t{
	unsigned short head;
	unsigned short len;
	int bal_angle;
	int bal_kp;
	int bal_kd;
	int vel_kp;
	int vel_ki;
	int enc_filte;
	int turn_kp;
	int turn_ki;
	int turn_cmd;
	unsigned short alen;
	unsigned short sum;
};

#define CMDLEN	sizeof(struct cmd_buff_t)
struct usart_buff_t{
	unsigned int 	index;
	unsigned char buff[2*CMDLEN];
};

#endif
