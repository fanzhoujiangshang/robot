#include "led_control.h"
#include "leds.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define UINT16_LEDMAX 50
static bool ledSetEnable = false;
static ledPWM_t ledPWM;
static ledPWM_t ledPWMSet={0, 0};


void ledControlInit(void)
{
	ledsInit();
}

bool ledControlTest(void)
{
	bool pass = true;

	pass &= ledsTest();

	return pass;
}

void ledControl(void)	/*功率输出控制*/
{
	if (ledSetEnable)
	{
		ledPWM = ledPWMSet;
		ledsSetRatio(LED_M1, ledPWM.m1);	/*控制电机输出百分比*/
		ledsSetRatio(LED_M2, ledPWM.m2);
		
//		printf("<powerControl....>\n");
	} else {
		ledsSetRatio(LED_M1, 0);	/*控制电机输出百分比*/
		ledsSetRatio(LED_M2, 0);	
		
// 		ledsSetRatio(LED_M1, 100);	/*控制电机输出百分比*/
//		ledsSetRatio(LED_M2, 100);	
//		printf("<ledControl....>\n");		
	}

}

void getLedPWM(ledPWM_t* get)
{
	*get = ledPWM;
}

void setLedPWM(u8 m1_set)
{
	ledSetEnable = 1;
	ledPWMSet.m1 = m1_set;
}
