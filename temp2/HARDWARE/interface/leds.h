#ifndef __LEDS_H
#define __LEDS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
#define LED_M1  		0
#define LED_M2  		1


void ledsInit(void);		/*led初始化*/
bool ledsTest(void);		/*led测试*/
void ledsSetRatio(u32 id, u16 ithrust);	/*设置led占空比*/

#endif /* __MOTORS_H */

