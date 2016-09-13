#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Set_Pwm(int moto1, int moto2);
int  Turn_Off(float angle, int voltage);
	
#endif
