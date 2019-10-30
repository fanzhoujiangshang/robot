#ifndef __NVIC_H
#define __NVIC_H
#include "sys.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/


void nvicInit(void);
u32 getSysTickCnt(void);	

#endif /* __NVIC_H */
