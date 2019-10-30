#include "sys.h"
#include "delay.h"
#include "leds.h"
#include "pm.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define DISABLE_LEDEN()	GPIO_ResetBits(GPIOC, GPIO_Pin_2);
#define ENABLE_LEDEN()   	GPIO_SetBits(GPIOC, GPIO_Pin_2);

#define DISABLE_LEDFAULT()	GPIO_ResetBits(GPIOC, GPIO_Pin_3);
#define ENABLE_LEDFAULT()   	GPIO_SetBits(GPIOC, GPIO_Pin_3);

static bool isInit = false;
const u32 LEDS[] = {LED_M1,LED_M2};

void ledsInit(void)	/*��ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//ʹ��PORTCʱ��  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  
	
	TIM_DeInit(TIM8);	//���³�ʼ��TIM8ΪĬ��״̬
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 	//PC6����ΪTIM8 CH1	LED1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 	//PC7 ����ΪTIM8 CH2	LED2

	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;	//
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//���ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//���츴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              				//��ʼ��PB6 7 10
	

  //����ʱ����0������199����Ϊ200�Σ�Ϊһ����ʱ����
  	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//����PSC��ֵΪ8400�������Ļ�ÿ����һ��Ϊ0.1ms   
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//�ظ���������
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);				//��ʼ��TIM8

  //����ʱ����0������199����Ϊ200�Σ�Ϊһ����ʱ����
//  	TIM_TimeBaseStructure.TIM_Period = 200 -1;   		//�Զ���װ��ֵ
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100KHz
//	TIM_TimeBaseStructure.TIM_Prescaler = 16800 - 1;//����PSC��ֵΪ8400�������Ļ�ÿ����һ��Ϊ0.1ms   	
//	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
//	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  	//��ʼ��TIM8 CH1����Ƚ�
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  	//��ʼ��TIM8 CH2����Ƚ�	
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM8��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���	

	TIM_ARRPreloadConfig(TIM8,ENABLE);	//TIM2	ARPEʹ�� 
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM2	
	TIM_CtrlPWMOutputs(TIM8, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л����� 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ENABLE_LEDEN()	

	isInit = true;
}

/*led����*/
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

/*����LED PWMռ�ձ�*/
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

