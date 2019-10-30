#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ���Ź���������	
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

#define WATCHDOG_RESET_MS 	150	/*���Ź���λʱ��*/
#define watchdogReset() 	(IWDG_ReloadCounter())


void watchdogInit(u16 xms);
bool watchdogTest(void);


#endif 

