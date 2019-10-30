#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"
#include "motor_control.h"

/********************************************************************************	 
 * ´´½¨ÈÕÆÚ:2018/6/22
 * °æ±¾£ºV1.2
 * °æÈ¨ËùÓÐ£¬µÁ°æ±Ø¾¿¡£
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

void motorsInit(void)	/*µç»ú³õÊ¼»¯*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);	//Ê¹ÄÜPORTE PORTDÊ±ÖÓ
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM1ºÍTIM4Ê±ÖÓÊ¹ÄÜ    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  
	
	TIM_DeInit(TIM4);	//ÖØÐÂ³õÊ¼»¯TIM4ÎªÄ¬ÈÏ×´Ì¬
	TIM_DeInit(TIM1);	//ÖØÐÂ³õÊ¼»¯TIM2ÎªÄ¬ÈÏ×´Ì¬
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 	//PE9 ¸´ÓÃÎªTIM1 CH1	MOTOR1
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 	//PE11 ¸´ÓÃÎªTIM1 CH2	MOTOR2
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 	//PE9 ¸´ÓÃÎªTIM1 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 	//PE11 ¸´ÓÃÎªTIM1 CH4	MOTOR4	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 	//PD14¸´ÓÃÎªTIM4 CH1	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 	//PD15 ¸´ÓÃÎªTIM4 CH2	MOTOR6
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); 	//PD14¸´ÓÃÎªTIM4 CH3	MOTOR5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 	//PD15 ¸´ÓÃÎªTIM4 CH4	MOTOR6	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;	//
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//¸´ÓÃ¹¦ÄÜ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//ËÙ¶È100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//ÍÆÍì¸´ÓÃÊä³ö
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//ÉÏÀ­
	GPIO_Init(GPIOE,&GPIO_InitStructure);              				//³õÊ¼»¯PB6 7 10
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;							//PA5
	GPIO_Init(GPIOD,&GPIO_InitStructure);              				//³õÊ¼»¯PA5		

  //µ±¶¨Ê±Æ÷´Ó0¼ÆÊýµ½199£¬¼´Îª200´Î£¬ÎªÒ»¸ö¶¨Ê±ÖÜÆÚ
  //	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
 	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;  //21400
	// Í¨ÓÃ¿ØÖÆ¶¨Ê±Æ÷Ê±ÖÓÔ´TIMxCLK = HCLK/2=84MHz 
	// Éè¶¨¶¨Ê±Æ÷ÆµÂÊÎª=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 90 - 1;//ÉèÖÃPSCµÄÖµÎª8400£¬ÕâÑùµÄ»°Ã¿¼ÆÊýÒ»´ÎÎª0.1ms   
	
//	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//×Ô¶¯ÖØ×°ÔØÖµ
//	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//¶¨Ê±Æ÷·ÖÆµ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//ÏòÉÏ¼ÆÊýÄ£Ê½	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//Ê±ÖÓ·ÖÆµ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//ÖØ¸´¼ÆÊý´ÎÊý
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//³õÊ¼»¯TIM4

  //µ±¶¨Ê±Æ÷´Ó0¼ÆÊýµ½199£¬¼´Îª200´Î£¬ÎªÒ»¸ö¶¨Ê±ÖÜÆÚ
 // 	TIM_TimeBaseStructure.TIM_Period = 200 -1;   
  	TIM_TimeBaseStructure.TIM_Period =  20000 - 1;  //
	// Í¨ÓÃ¿ØÖÆ¶¨Ê±Æ÷Ê±ÖÓÔ´TIMxCLK = HCLK/2=84MHz 
	// Éè¶¨¶¨Ê±Æ÷ÆµÂÊÎª=TIMxCLK/(TIM_Prescaler+1)=100KHz
 	TIM_TimeBaseStructure.TIM_Prescaler = 180 - 1;//ÉèÖÃPSCµÄÖµÎª8400£¬ÕâÑùµÄ»°Ã¿¼ÆÊýÒ»´ÎÎª0.1ms   	
// 	TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//ÉèÖÃPSCµÄÖµÎª8400£¬ÕâÑùµÄ»°Ã¿¼ÆÊýÒ»´ÎÎª0.1ms 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);				//³õÊ¼»¯TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMÄ£Ê½1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//Ê¹ÄÜÊä³ö
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//¸ßµçÆ½ÓÐÐ§
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//¿ÕÏÐ¸ßµçÆ½	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM4 CH2Êä³ö±È½Ï
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM4 CH1Êä³ö±È½Ï	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM4 CH2Êä³ö±È½Ï
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM4 CH1Êä³ö±È½Ï
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM2 CH3Êä³ö±È½Ï
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM2 CH1Êä³ö±È½Ï
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM2 CH3Êä³ö±È½Ï
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  	//³õÊ¼»¯TIM2 CH1Êä³ö±È½Ï	

	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM4ÔÚCCR2ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM4ÔÚCCR1ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM4ÔÚCCR2ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM4ÔÚCCR1ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM2ÔÚCCR3ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM2ÔÚCCR1ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM2ÔÚCCR3ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //Ê¹ÄÜTIM2ÔÚCCR1ÉÏµÄÔ¤×°ÔØ¼Ä´æÆ÷	
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPEÊ¹ÄÜ 
	TIM_ARRPreloadConfig(TIM1,ENABLE);	//TIM2	ARPEÊ¹ÄÜ 
	
	TIM_Cmd(TIM4, ENABLE);  //Ê¹ÄÜTIM4
	TIM_Cmd(TIM1, ENABLE);  //Ê¹ÄÜTIM2	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//Ê¹ÄÜTIM1µÄPWMÊä³ö£¬TIM1ÓëTIM8ÓÐÐ§,Èç¹ûÃ»ÓÐÕâÐÐ»áÎÊÌâ 


	/*¨º1?¨¹led¨º¡À?¨®*/
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

/*µç»ú²âÊÔ*/
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

/*ÉèÖÃµç»úPWMÕ¼¿Õ±È*/
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

