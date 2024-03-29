#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"
#include "motor_control.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define MOTOR_POWER		PAout(5)

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0, 0, 0, 0, 0};
const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4, MOTOR_M5, MOTOR_M6, MOTOR_M7, MOTOR_M8 };
extern control_radio_t control_radio;

//static u16 ratioToCCRx(u16 val)
//{
//	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
//}

void motor_power_enable(void)
{	
	MOTOR_POWER = 1;	
}

void motor_power_disable(void)
{	
	MOTOR_POWER = 0;	
}

void motorsInit(void)	/*电机初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);	//使能PORTE PORTD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM1和TIM4时钟使能    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  
	
	TIM_DeInit(TIM4);	//重新初始化TIM4为默认状态
	TIM_DeInit(TIM1);	//重新初始化TIM2为默认状态
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 	//PE9 复用为TIM1 CH1	MOTOR1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 	//PE11 复用为TIM1 CH2	MOTOR2
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 	//PE9 复用为TIM1 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 	//PE11 复用为TIM1 CH4	MOTOR4	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 	//PD14复用为TIM4 CH1	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 	//PD15 复用为TIM4 CH2	MOTOR6
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); 	//PD14复用为TIM4 CH3	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 	//PD15 复用为TIM4 CH4	MOTOR6	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;	//
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//速度100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              				//初始化PB6 7 10
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;							//PA5
	GPIO_Init(GPIOD,&GPIO_InitStructure);              				//初始化PA5		

  //当定时器从0计数到199，即为200次，为一个定时周期
  //	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
 	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;  //21400
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 90 - 1;//设置PSC的值为8400，这样的话每计数一次为0.1ms   
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//自动重装载值
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//重复计数次数
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//初始化TIM4

  //当定时器从0计数到199，即为200次，为一个定时周期
 // 	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
  	TIM_TimeBaseStructure.TIM_Period =  20000 - 1;  //
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 180 - 1;//设置PSC的值为8400，这样的话每计数一次为0.1ms   	
// 	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//设置PSC的值为8400，这样的话每计数一次为0.1ms 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);				//初始化TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//高电平有效
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//空闲高电平	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  	//初始化TIM4 CH2输出比较
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  	//初始化TIM4 CH1输出比较	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  	//初始化TIM4 CH2输出比较
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  	//初始化TIM4 CH1输出比较
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM2 CH3输出比较
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM2 CH1输出比较
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM2 CH3输出比较
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM2 CH1输出比较	

	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存�
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器	
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPE使能 
	TIM_ARRPreloadConfig(TIM1,ENABLE);	//TIM2	ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	TIM_Cmd(TIM1, ENABLE);  //使能TIM2	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题 


	/*ê1?üledê±?ó*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
	 GPIO_Init(GPIOA, &GPIO_InitStructure);	
	MOTOR_POWER = 1;	   
	
	
//	TIM_SetCompare1(TIM1,1500);  	//PE9  4
//	TIM_SetCompare2(TIM1,1500);		//PE11 3
//	TIM_SetCompare3(TIM1,1500);		//PE13 2
//	TIM_SetCompare4(TIM1,1500);		//PE14 1
//	TIM_SetCompare1(TIM4,1500);		//PD12 5
//	TIM_SetCompare2(TIM4,1500);   //PD13 6
//	TIM_SetCompare3(TIM4,1500);		//PD14 7
//	TIM_SetCompare4(TIM4,1500);		//PD15

	control_radio.gas 		= 500;
	control_radio.pitch 	= 500;
	control_radio.roll 		= 500;
	control_radio.yaw 		= 500;
	control_radio.forward = 500;
	control_radio.lateral = 500;
	isInit = true;
}

/*电机测试*/
bool motorsTest(void)
{
	int i;

	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
//	       printf("<motorsTest %d>\n",MOTORS[i]);	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		delay_xms(MOTORS_TEST_ON_TIME_MS);//MOTORS_TEST_ON_TIME_MS
	}

	return isInit;
}

/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
//		u16 ratio=ithrust+1;
		u16 ratio=ithrust;

	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		float thrust = ((float)ithrust / 65536.0f) * 60;
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
		float supply_voltage = pmGetBatteryVoltage();
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		motor_ratios[id] = ratio;		1
	#endif
		switch(id)
		{
			case 1:		/*MOTOR_M1*/
				TIM_SetCompare4(TIM1,ratio);
			
				//TIM_SetCompare3(TIM4,ratio);
				break;
			case 2:		/*MOTOR_M2*/
				TIM_SetCompare3(TIM1,ratio);
				break;
			case 3:		/*MOTOR_M3*/
				TIM_SetCompare2(TIM1,ratio);
				break;
			case 4:		/*MOTOR_M4*/
				TIM_SetCompare1(TIM1,ratio);
				break;
				
			case 5:		/*MOTOR_M5*/
				TIM_SetCompare1(TIM4,ratio);
				break;
			case 6:		/*MOTOR_M6*/	
				TIM_SetCompare2(TIM4,ratio);
				break;
			case 7:		/*MOTOR_M5*/
				TIM_SetCompare3(TIM4,ratio);
			
				//TIM_SetCompare4(TIM1,ratio);
				break;
			case 8:		/*MOTOR_M6*/	
				TIM_SetCompare4(TIM4,ratio);
				break;				
			default: break;
		}
	}
}

