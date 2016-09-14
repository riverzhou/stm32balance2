#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "clock.h"
#include "usart.h"
#include "ioi2c.h"
#include "MPU6050.h"
#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "control.h"
#include "led.h"
#include "main.h"
#include "command.h"

struct env_t g_env;
struct env_t *ENV = &g_env;

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	while(1);
}
#endif

void main_Init(void)
{
	memset(ENV, 0, sizeof(struct env_t));
	
	ENV->env_lock			= 0;				// 全局变量锁
	ENV->bal_angel		= 0;				// 平衡中值 	// 放大1000倍
	ENV->bal_kp				= 300;			// 平衡KP
	ENV->bal_kd				= 1000;			// 平衡KD  		// 放大1000倍
	ENV->vel_kp				= 80;				// 速度KP
	ENV->vel_ki				= 400;			// 速度KI			// 放大1000倍
	ENV->enc_filte		= 800;			// 编码滤波		// 放大1000倍
}

/*
SysTick_IRQn	0:0
EXTI9_5_IRQn	2:0
USART1_IRQn		3:0
USART3_IRQn		4:0
*/
void nvic_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); // 配置中断模式为3:1 (0-7:0-1)
}

void second(void)
{
	static volatile unsigned int count = 0;
	if(SYS_ClockTick >= count){
			count += 1000;

			printf("\
SYS_C %-8u , \
mpu_c %-8u , \
mpu_b_a %-8d , \
mpu_b_g %-8d , \
mpu_t_g %-8d , \
enc_l %-8d , \
enc_r %-8d , \
mot_l %-8d , \
mot_r %-8d , \
bat_v	%-8d \
\r\n", 
SYS_ClockTick, 
ENV->mpu_count,
ENV->mpu_bal_angle,
ENV->mpu_bal_gypo,
ENV->mpu_turn_gypo,
ENV->enc_left,
ENV->enc_right,
ENV->moto_left,
ENV->moto_right,
ENV->bat_voltage
);

			for(int i=0; i<CMDLEN; i++){
				printf("%2X ", CMD->buff[i]);
			}
			printf("\t[P]%2u\r\n", CMD->index);
			
			ENV->bat_voltage = Get_battery_volt();
	}	
}

int main(void)
{
	main_Init();
	command_init();
	nvic_init();									//=====初始化中断模式
	clock_Init();                 //=====初始化软时钟
	usart_Init();									//=====初始化串口
	LED_Init();										//=====初始化LED
	Battery_Init();								//=====初始电池电压监控
	Encoder_Init();            		//=====初始化电机编码器接口
	Motor_Init();   							//=====初始化驱动电机
	IIC_Init();                   //=====初始化模拟IIC
	DMP_Init();                   //=====初始化DMP
	control_Init();								//=====初始化控制中断，5ms
	
	printf("hello world \r\n");

	while(1)
		second();
}
