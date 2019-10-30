#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

typedef struct 
{
	u8 m1;
	u8 m2;		
}ledPWM_t;

void ledControlInit(void);
bool ledControlTest(void);
void ledControl(void);

void getLedPWM(ledPWM_t* get);
void setLedPWM(u8 m1_set);
#endif 
