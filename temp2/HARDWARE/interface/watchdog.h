#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 看门狗驱动代码	
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

#define WATCHDOG_RESET_MS 	150	/*看门狗复位时间*/
#define watchdogReset() 	(IWDG_ReloadCounter())


void watchdogInit(u16 xms);
bool watchdogTest(void);


#endif 

