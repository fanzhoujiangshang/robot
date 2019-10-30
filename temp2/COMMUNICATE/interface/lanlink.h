#ifndef __LAN_H
#define __LAN_H
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

void lanlinkInit(void);
void lanlinkRxTask(void *param);
void lanlinkTxTask(void *param);
bool lanlinkSendPacket(const atkp_t *p);
bool lanlinkSendPacketBlocking(const atkp_t *p);
bool lanlinkAckPacket(const atkp_t *p);
int lanlinkGetFreeTxQueuePackets(void);

#endif /*__LAN_H */

