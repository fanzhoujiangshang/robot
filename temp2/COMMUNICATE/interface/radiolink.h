#ifndef __RADIO_H
#define __RADIO_H
#include <stdint.h>
#include <stdbool.h>
#include "atkp.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

void radiolinkInit(void);
void radiolinkRxTask(void *param);
void radiolinkTxTask(void *param);
bool radiolinkSendPacket(const atkp_t *p);
bool radiolinkSendPacketBlocking(const atkp_t *p);
bool radiolinkAckPacket(const atkp_t *p);
int radiolinkGetFreeTxQueuePackets(void);

#endif /*__RADIO_H */

