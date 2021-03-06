#ifndef __DELAY_H
#define __DELAY_H 			   

void clock_Init(void);

void delay_ms(unsigned int nms);
void delay_us(unsigned int nus);
unsigned int get_ms(unsigned long *count);

extern volatile unsigned int SYS_ClockTick;
extern volatile int SYS_ClockDelay;

#endif
