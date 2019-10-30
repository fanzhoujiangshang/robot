#include "sys.h"
#include "delay.h"
#include "leds.h"
#include "pm.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define DISABLE_LEDEN()	GPIO_ResetBits(GPIOC, GPIO_Pin_2);
#define ENABLE_LEDEN()   	GPIO_SetBits(GPIOC, GPIO_Pin_2);

#define DISABLE_LEDFAULT()	GPIO_ResetBits(GPIOC, GPIO_Pin_3);
#define ENABLE_LEDFAULT()   	GPIO_SetBits(GPIOC, GPIO_Pin_3);

static bool isInit = false;
const u32 LEDS[] = {LED_M1,LED_M2};

void ledsInit(void)	/*初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//使能PORTC时钟  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  
	
	TIM_DeInit(TIM8);	//重新初始化TIM8为默认状态
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 	//PC6复用为TIM8 CH1	LED1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 	//PC7 复用为TIM8 CH2	LED2

	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;	//
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//速度100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              				//初始化PB6 7 10
	

  //当定时器从0计数到199，即为200次，为一个定时周期
  	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//设置PSC的值为8400，这样的话每计数一次为0.1ms   
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//重复计数次数
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);				//初始化TIM8

  //当定时器从0计数到199，即为200次，为一个定时周期
//  	TIM_TimeBaseStructure.TIM_Period = 200 -1;   		//自动重装载值
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
//	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;//设置PSC的值为8400，这样的话每计数一次为0.1ms   	
//	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);				//初始化TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2;				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
//	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//高电平有效
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//空闲高电平	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  	//初始化TIM8 CH1输出比较
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  	//初始化TIM8 CH2输出比较	
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM8在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM8在CCR2上的预装载寄存器	

	TIM_ARRPreloadConfig(TIM8,ENABLE);	//TIM2	ARPE使能 
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM2	
	TIM_CtrlPWMOutputs(TIM8, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ENABLE_LEDEN()	

	isInit = true;
}

/*led测试*/
bool ledsTest(void)
{
    printf("ledsTest>\n");	
	int i = 0;
//	while(1)
	{
		ledsSetRatio(LEDS[0], i);
		  printf("ledsTest>%d\n",i);	
		delay_ms(5000);
		ledsSetRatio(LEDS[1], i);
		i++;
		i++;
		if(i==255) i=0;
	}

	return isInit;
}

/*设置LED PWM占空比*/
void ledsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;

		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare1(TIM8,ratio);
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare2(TIM8,ratio);
				break;				
			default: break;
		}
	}
}

