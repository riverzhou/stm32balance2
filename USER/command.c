#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "command.h"
#include "main.h"

struct usart_buff_t usart_buff;
struct usart_buff_t *CMD_RAW = &usart_buff;

/*
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
*/

int cmd_proc(unsigned char * buff)
{
	struct cmd_buff_t *cmd_p = (struct cmd_buff_t*)buff;

	if(cmd_p->head != 0xff || cmd_p->len != CMDLEN || (cmd_p->alen & cmd_p->len))
		return -1;

	int sum = 0;
	for(int i = 0; i < CMDLEN - 2; i++)
		sum += buff[i];
	if(cmd_p->sum != sum)
		return -1;

	if(ENV->env_lock)		//===如果环境变量区被锁住则不处理命令（打断了低级别的锁）
		return 0;
	ENV->env_lock = 1;	// 锁住环变量境区

	if(cmd_p->bal_angle !=0)
		ENV->bal_angle = cmd_p->bal_angle;

	if(cmd_p->bal_kp !=0)
		ENV->bal_kp = cmd_p->bal_kp;

	if(cmd_p->vel_kp !=0)
		ENV->vel_kp = cmd_p->vel_kp;

	if(cmd_p->vel_ki !=0)
		ENV->vel_ki = cmd_p->vel_ki;

	if(cmd_p->enc_filte !=0)
		ENV->enc_filte = cmd_p->enc_filte;

	ENV->env_lock = 0;	// 解锁环境变量区

	return 0;
}

void command_proc(unsigned char data)
{	
	CMD_RAW->index += 1;
	CMD_RAW->index %= CMDLEN;
	CMD_RAW->buff[CMD_RAW->index] = data;
	CMD_RAW->buff[CMD_RAW->index+CMDLEN] = data;
	cmd_proc(&(CMD_RAW->buff[CMD_RAW->index+1]));
} 

void command_init(void)
{
	memset(CMD_RAW, 0, sizeof(struct usart_buff_t));
	CMD_RAW->index = CMDLEN-1;
}

