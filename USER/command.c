#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "clock.h"
#include "command.h"
#include "main.h"
#include "log.h"

struct usart_buff_t  usart_buff;
struct usart_buff_t* usart_buff_p = &usart_buff;

#define CMD_TIMEOUT	50		// 50ms无数据则允许其他同级别中断打断

void cmd_setbusy(void)
{
		SYS_ClockDelay = CMD_TIMEOUT;
}

int cmd_isbusy(void)
{
	return (SYS_ClockDelay>0);
}

/*
struct cmd_t{
        int bal_angle;
        int bal_kp;
        int bal_kd;
        int vel_kp;
        int vel_ki;
        int enc_filte;
        int turn_kp;
        int turn_ki;
        int turn_cmd;
};

struct cmd_buff_t{
        unsigned short head;
        unsigned short len;
        struct cmd_t cmd;
        unsigned short alen;
        unsigned short sum;
};
*/

int cmd_proc(unsigned char * buff)
{
	struct cmd_buff_t *cmd_p = (struct cmd_buff_t*)buff;

	if(cmd_p->head != 0xff || cmd_p->len != CMDLEN || (cmd_p->len & cmd_p->alen))
		return -1;

	int sum = 0;
	for(int i = 0; i < CMDLEN - 2; i++)
		sum += buff[i];
	if(cmd_p->sum != sum)
		return -1;

	if(ENV->env_lock)		//===如果环境变量区被锁住则不处理命令（打断了低级别的锁）
		return 0;
	ENV->env_lock = 1;	// 锁住环变量境区

	//if(cmd_p->cmd.bal_angle !=0)
	ENV->bal_angle = cmd_p->cmd.bal_angle;

	if(cmd_p->cmd.bal_kp !=0)
		ENV->bal_kp = cmd_p->cmd.bal_kp;

	if(cmd_p->cmd.bal_kd !=0)
		ENV->bal_kd = cmd_p->cmd.bal_kd;

	if(cmd_p->cmd.vel_kp !=0)
		ENV->vel_kp = cmd_p->cmd.vel_kp;

	if(cmd_p->cmd.vel_ki !=0)
		ENV->vel_ki = cmd_p->cmd.vel_ki;

	if(cmd_p->cmd.enc_filte !=0)
		ENV->enc_filte = cmd_p->cmd.enc_filte;

	ENV->env_lock = 0;	// 解锁环境变量区

	command_buff_reset();
	
	return 0;
}

void command_proc(unsigned char data)
{	
	cmd_setbusy();											// 防止同级别中断打断导致丢数据
	
	usart_buff_p->index += 1;
	usart_buff_p->index %= CMDLEN;
	usart_buff_p->buff[usart_buff_p->index] = data;
	usart_buff_p->buff[usart_buff_p->index+CMDLEN] = data;
	
	LOG_D("%.2X ",data);
	
	cmd_proc(&(usart_buff_p->buff[usart_buff_p->index+1]));
} 

void command_buff_reset(void)
{
	memset(usart_buff_p, 0, sizeof(struct usart_buff_t));
	usart_buff_p->index = CMDLEN-1;
}

