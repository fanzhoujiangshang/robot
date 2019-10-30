#ifndef __CONSOLE_H
#define __CONSOLE_H
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C)
 * All rights reserved
********************************************************************************/

void consoleInit(void);
bool consoleTest(void);
int consolePutchar(int ch);	/* 输入一个字符到console缓冲区*/
int consolePutcharFromISR(int ch);	/* 中断方式输入一个字符到console缓冲区*/
int consolePuts(char *str);	/* 输入一个字符串到console缓冲区*/

#endif /*__CONSOLE_H*/
