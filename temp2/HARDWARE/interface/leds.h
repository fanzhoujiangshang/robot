#ifndef __LEDS_H
#define __LEDS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define LED_M1  		0
#define LED_M2  		1


void ledsInit(void);		/*led��ʼ��*/
bool ledsTest(void);		/*led����*/
void ledsSetRatio(u32 id, u16 ithrust);	/*����ledռ�ձ�*/

#endif /* __MOTORS_H */

