#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
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
