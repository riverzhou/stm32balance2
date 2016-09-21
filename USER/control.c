#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f10x.h"
#include "control.h"
#include "MPU6050.h"
#include "encoder.h"
#include "motor.h"
#include "battery.h"
#include "ioi2c.h"
#include "clock.h"
#include "main.h"
#include "command.h"

#define ZERO_LIMIT		1.0E-19
#define PI         		3.1415926535897932

#define BAL_VOLTAGE		ENV->bat_voltage

#define MPU_COUNT			ENV->mpu_count
#define MPU_BAL_ANGLE ENV->mpu_bal_angle
#define MPU_BAL_GYRO 	ENV->mpu_bal_gypo
#define MPU_TURN_GYPO	ENV->mpu_turn_gypo

#define ENC_LEFT			ENV->enc_left
#define ENC_RIGHT			ENV->enc_right

#define MOTO_LEFT			ENV->moto_left
#define MOTO_RIGHT		ENV->moto_right

/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/

void control_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);					//外部中断，需要使能AFIO时钟
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin  	= GPIO_Pin_5;	           				//端口配置
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;         				//上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      				//根据设定参数初始化GPIOB 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
	
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line		= EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;				//下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 															//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	memset(&NVIC_InitStructure,0,sizeof(NVIC_InitStructure));
	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI9_5_IRQn;						//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;			//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;						//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 	
}

/**************************************************************************
函数功能：获取平衡角度和平衡角速度和转向角速度
入口参数：衡角度和平衡角速度和转向角速度
返回  值：无
**************************************************************************/
void Get_Angle(float* Bal_Angle, float* Bal_Gyro, float* Turn_Gyro)
{ 
	float Pitch, Gyro_Y, Gyro_Z;

	if(!Read_DMP(&Pitch, &Gyro_Y, &Gyro_Z)){
		*Bal_Angle = Pitch;
		*Bal_Gyro  = Gyro_Y;
		*Turn_Gyro = Gyro_Z;
	}
	else{
		*Bal_Angle = 0;
		*Bal_Gyro  = 0;
		*Turn_Gyro = 0;
	}
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{ 
	float BAL_ANGLE = (Angle - (float)ENV->bal_angle/1000)*PI/180;
	float BAL_KP		=	(float)ENV->bal_kp;
	float BAL_KD		= (float)ENV->bal_kd/1000;
	float sin_angle = sin(BAL_ANGLE);
	float cos_angle = cos(BAL_ANGLE);
	if(cos_angle < ZERO_LIMIT) cos_angle = ZERO_LIMIT;
	return (BAL_KP * sin_angle + Gyro * BAL_KD / cos_angle) / cos_angle;
  //===求出平衡的角度中值 和机械相关
	//===计算平衡控制的电机PWM  PD控制
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	static float Encoder_Least=0.0f, Encoder=0.0f, Encoder_Integral=0.0f;
	float ENC_FILTE = (float)ENV->enc_filte/1000;
	float VEL_KP 		=	(float)ENV->vel_kp;
	float VEL_KI 		=	(float)ENV->vel_ki/1000;
	//=============速度PI控制器=======================//	
	Encoder_Least = (encoder_left + encoder_right) - 0;               		//===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= ENC_FILTE;		                                        			//===一阶低通滤波器       
	Encoder += Encoder_Least * (1-ENC_FILTE);	                          	//===一阶低通滤波器    
	Encoder_Integral += Encoder;                                      		//===积分出位移 积分时间：10ms
	//Encoder_Integral = Encoder_Integral - Movement;                   	//===接收遥控器数据，控制前进后退
	if(Encoder_Integral>10000)  Encoder_Integral = 10000;            			//===积分限幅
	if(Encoder_Integral<-10000)	Encoder_Integral = -10000;            		//===积分限幅	
	return Encoder * VEL_KP + Encoder_Integral * VEL_KI;              		//===速度控制	
}

/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/

int EXTI9_5_IRQHandler(void) 
{ 
	MPU_COUNT++;
	
	static int Flag_Target = 0;
	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
		EXTI_ClearFlag(EXTI_Line5);																					//清除LINE5上的中断标志位   
		
		if(cmd_isbusy())																										//防止中断CMD通讯
			return 0;
		
		float Bal_Angle=0.0f, Bal_Gyro=0.0f, Turn_Gyro=0.0f;								//平衡倾角 平衡陀螺仪 转向陀螺仪
		Get_Angle(&Bal_Angle, &Bal_Gyro, &Turn_Gyro);                       //===更新姿态	
				
		Flag_Target=!Flag_Target;
		if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			return 0;	                                               

		if(ENV->env_lock)																										//===如果环境变量区被锁住则不处理命令（打断了低级别的锁）
			return 0;
		ENV->env_lock = 1;																									//===锁住环境变量区
		
		int Encoder_Left  = -Read_Encoder(2);                               //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		int Encoder_Right = Read_Encoder(4);                                //===读取编码器的值
		int Balance_Pwm   = balance(Bal_Angle, Bal_Gyro);                		//===平衡PID控制	
		int Velocity_Pwm  = velocity(Encoder_Left, Encoder_Right);          //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		int Motor1 = Balance_Pwm - Velocity_Pwm;                            //===计算左轮电机最终PWM
		int Motor2 = Balance_Pwm - Velocity_Pwm;                            //===计算右轮电机最终PWM

		if(!Turn_Off(Bal_Angle, BAL_VOLTAGE))                   					  //===检测倾角是否在许可范围内
			Set_Pwm(Motor1, Motor2);                                          //===赋值给PWM寄存器  

		MPU_BAL_ANGLE = (int)(Bal_Angle*1000);
		MPU_BAL_GYRO 	= (int)(Bal_Gyro*1000);
		MPU_TURN_GYPO	= (int)(Turn_Gyro*1000);
		ENC_LEFT			=	Encoder_Left;
		ENC_RIGHT			=	Encoder_Right;
		MOTO_LEFT			=	Motor1;
		MOTO_RIGHT		=	Motor2;
		
		ENV->env_lock = 0;																									//解锁环境变量区
	}       	
	return 0;	  
} 
