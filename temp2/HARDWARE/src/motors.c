#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"
#include "motor_control.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
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

void motorsInit(void)	/*�����ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);	//ʹ��PORTE PORTDʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM1��TIM4ʱ��ʹ��    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  
	
	TIM_DeInit(TIM4);	//���³�ʼ��TIM4ΪĬ��״̬
	TIM_DeInit(TIM1);	//���³�ʼ��TIM2ΪĬ��״̬
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 	//PE9 ����ΪTIM1 CH1	MOTOR1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 	//PE11 ����ΪTIM1 CH2	MOTOR2
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 	//PE9 ����ΪTIM1 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 	//PE11 ����ΪTIM1 CH4	MOTOR4	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 	//PD14����ΪTIM4 CH1	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 	//PD15 ����ΪTIM4 CH2	MOTOR6
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); 	//PD14����ΪTIM4 CH3	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 	//PD15 ����ΪTIM4 CH4	MOTOR6	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;	//
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//���ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//���츴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//����
	GPIO_Init(GPIOE,&GPIO_InitStructure);              				//��ʼ��PB6 7 10
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;							//PA5
	GPIO_Init(GPIOD,&GPIO_InitStructure);              				//��ʼ��PA5		

  //����ʱ����0������199����Ϊ200�Σ�Ϊһ����ʱ����
  //	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
 	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;  //21400
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 90 - 1;//����PSC��ֵΪ8400�������Ļ�ÿ����һ��Ϊ0.1ms   
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//�ظ���������
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//��ʼ��TIM4

  //����ʱ����0������199����Ϊ200�Σ�Ϊһ����ʱ����
 // 	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
  	TIM_TimeBaseStructure.TIM_Period =  20000 - 1;  //
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 180 - 1;//����PSC��ֵΪ8400�������Ļ�ÿ����һ��Ϊ0.1ms   	
// 	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//����PSC��ֵΪ8400�������Ļ�ÿ����һ��Ϊ0.1ms 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�	

	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ��
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���	
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPEʹ�� 
	TIM_ARRPreloadConfig(TIM1,ENABLE);	//TIM2	ARPEʹ�� 
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM2	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л����� 


	/*��1?��led����?��*/
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

/*�������*/
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

/*���õ��PWMռ�ձ�*/
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

