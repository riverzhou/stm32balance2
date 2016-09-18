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
#include "log.h"

struct env_t g_env;
struct env_t *ENV = &g_env;

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	while(1) __NOP();
}
#endif

void main_Init(void)
{
	memset(ENV, 0, sizeof(struct env_t));

	ENV->env_lock			= 0;				// 全局变量锁
	ENV->bal_angle		= 0;				// 平衡中值 	// 放大1000倍
	ENV->bal_kp				= 300;			// 平衡KP
	ENV->bal_kd				= 1000;			// 平衡KD  		// 放大1000倍
	ENV->vel_kp				= 80;				// 速度KP
	ENV->vel_ki				= 400;			// 速度KI			// 放大1000倍
	ENV->enc_filte		= 800;			// 编码滤波		// 放大1000倍
}

/*
SysTick_IRQn	0:0
EXTI9_5_IRQn	2:1
USART1_IRQn		2:2
USART3_IRQn		2:0
*/
void nvic_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置中断模式为2:2 (0-3:0-3)
}

void output_env(void)
{
	unsigned char buff[sizeof(struct env_buff_t)] = {0};
	struct env_buff_t* buff_p = (struct env_buff_t*)buff;
	
	if(ENV->env_lock)		//===如果环境变量区被锁住则不处理命令（打断了低级别的锁）
		return;
	ENV->env_lock = 1;	// 锁住环变量境区
	
	memcpy(&(buff_p->env),ENV,sizeof(struct env_t));
	ENV->env_lock = 0;	// 解锁环变量境区

	buff_p->head = 0xff;
	buff_p->len  = sizeof(struct env_buff_t);
	buff_p->alen = ~buff_p->len;
	
	int sum=0;
	for(int i=0;i<sizeof(struct env_buff_t)-2;i++){
		sum += buff[i];
	}
	buff_p->sum = sum;
	
	for(int i=0;i<sizeof(struct env_buff_t);i++)
		usart3_PutChar(buff[i]);
}

void second(void)
{
	static volatile unsigned int count = 0;
	if(SYS_ClockTick >= count){
			count += 1000;
			LOG_D("SYS_ClockTick %u\r\n",SYS_ClockTick);
			output_env();
			ENV->bat_voltage = Get_battery_volt();
	}	
}

int main(void)
{
	main_Init();
	command_buff_reset();
	nvic_init();		//=====初始化中断模式
	clock_Init();		//=====初始化软时钟
	usart1_Init();	//=====初始化串口1
	usart3_Init();	//=====初始化串口3
	Battery_Init();	//=====初始电池电压监控
	Encoder_Init();	//=====初始化电机编码器接口
	Motor_Init();		//=====初始化驱动电机
	IIC_Init();			//=====初始化模拟IIC
	DMP_Init();			//=====初始化DMP
	control_Init();	//=====初始化控制中断，5ms
	LED_Init();			//=====初始化LED

	printf("hello world \r\n");

	while(1)
		second();
}
