#ifndef __CONSOLE_H
#define __CONSOLE_H
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C)
 * All rights reserved
********************************************************************************/

void consoleInit(void);
bool consoleTest(void);
int consolePutchar(int ch);	/* ����һ���ַ���console������*/
int consolePutcharFromISR(int ch);	/* �жϷ�ʽ����һ���ַ���console������*/
int consolePuts(char *str);	/* ����һ���ַ�����console������*/

#endif /*__CONSOLE_H*/
