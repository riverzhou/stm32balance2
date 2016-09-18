#ifndef _COMMAND_H_
#define _COMMAND_H_

extern void command_proc(unsigned char data);
extern void command_buff_reset(void);
extern int cmd_isbusy(void);

#endif
