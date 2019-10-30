#ifndef __USBLINK_H
#define __USBLINK_H
#include <stdbool.h>
#include "atkp.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

void usblinkInit(void);
bool usblinkSendPacket(const atkp_t *p);
bool usblinkAckPacket(const atkp_t *p);
int usblinkGetFreeTxQueuePackets(void);
void usblinkRxTask(void *param);
void usblinkTxTask(void *param);


#endif /*usblink.h*/

