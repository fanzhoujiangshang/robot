#include "comm.h"
#include "config.h"
#include "console.h"
#include "radiolink.h"
#include "usblink.h"
#include "lanlink.h"


/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

static bool isInit;

void commInit(void)
{
	if (isInit) return;
	radiolinkInit();	/*无线通信初始化*/
	usblinkInit();		/*USB通信初始化*/
	lanlinkInit();

	isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= consoleTest();
  
  return pass;
}

