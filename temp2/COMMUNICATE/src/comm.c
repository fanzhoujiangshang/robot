#include "comm.h"
#include "config.h"
#include "console.h"
#include "radiolink.h"
#include "usblink.h"
#include "lanlink.h"


/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

static bool isInit;

void commInit(void)
{
	if (isInit) return;
	radiolinkInit();	/*����ͨ�ų�ʼ��*/
	usblinkInit();		/*USBͨ�ų�ʼ��*/
	lanlinkInit();

	isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= consoleTest();
  
  return pass;
}

