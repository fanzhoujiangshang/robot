#include <string.h>
#include "config.h"
#include "lanlink.h"
#include "config_param.h"
#include "tcp_server.h" 

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

#define LANLINK_TX_QUEUE_SIZE  30 /*接收队列个数*/
#if 1
static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;
#endif
static bool isInit;
static atkp_t txPacket;
static atkp_t rxPacket;
static xQueueHandle  lantxQueue;
static xQueueHandle  lanackQueue;

static void lanSendPacket(atkp_t *p);

//radiolink接收ATKPPacket任务
void lanlinkRxTask(void *param)
{
	rxState = waitForStartByte1;
	
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;

	while(1)
	{
		if (tcpslkGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;				
					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;					
					break;
				case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						cksum += c;						
					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;					
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{		
						atkpReceivePacketBlocking(&rxPacket, 	LAN_TYPE);		
						vTaskDelay(50);
						/*接收到一个数据包则发送一个包*/
						if(xQueueReceive(lanackQueue, &txPacket, 0) == pdTRUE)
						{
							ASSERT(txPacket.dataLen <= ATKP_MAX_DATA_SIZE);
//							printf("lanSendPacket......\n");							

							lanSendPacket(&txPacket);
						}else{
//							printf("lanSendPacketis null \n");
						}								
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
						printf("C = %0x ,CHECK = %02x \n",c,cksum);	
					}
					rxState = waitForStartByte1;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStartByte1;
		}	
	}
}


void lanlinkInit(void)
{
	if (isInit) return;
	
	 tcpserverInit();	
	 
	/*创建发送队列，CRTP_TX_QUEUE_SIZE个消息*/
	lantxQueue = xQueueCreate(LANLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(lantxQueue);

	lanackQueue = xQueueCreate(LANLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(lanackQueue);
	
	isInit = true;
}

/*打包ATKPPacket数据发送*/
static void lanSendPacket(atkp_t *p)
{
	int dataSize;
	u8 cksum = 0;
	u8 sendBuffer[36];
	
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);

	sendBuffer[0] = UP_BYTE1;
	sendBuffer[1] = UP_BYTE2;
	sendBuffer[2] = p->msgID;
	sendBuffer[3] = p->dataLen;
	
	memcpy(&sendBuffer[4], p->data, p->dataLen);
	dataSize = p->dataLen + 5;//加上cksum
	/*计算校验和*/
	for (int i=0; i<dataSize-1; i++)
	{
		cksum += sendBuffer[i];
	}
	sendBuffer[dataSize-1] = cksum;
	
	TcpServerSendDataBlocking(dataSize, sendBuffer);
}

// radiolink发送ATKPPacket任务
void lanlinkTxTask(void *param)
{
	atkp_t p;
	u8 sendBuffer[64];
	u8 cksum;
	u8 dataLen;
	while(1)
	{
		xQueueReceive(lantxQueue, &p, portMAX_DELAY);
		
		sendBuffer[0] = UP_BYTE1;
		sendBuffer[1] = UP_BYTE2;
		sendBuffer[2] = p.msgID;
		sendBuffer[3] = p.dataLen;
		memcpy(&sendBuffer[4], p.data, p.dataLen);
		cksum = 0;
		for (int i = 0; i < p.dataLen+4; i++)
		{
			cksum += sendBuffer[i];
		}
		dataLen = p.dataLen + 5;
		sendBuffer[dataLen - 1] = cksum;
		
	       TcpServerSendDataBlocking(dataLen, sendBuffer);
	}
}


bool lanlinkSendPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(lantxQueue, p, 0);
}

bool lanlinkSendPacketBlocking(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(lantxQueue, p, portMAX_DELAY);	
}

bool lanlinkAckPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(lanackQueue, p, 0);
}

//获取剩余可用txQueue个数
int lanlinkGetFreeTxQueuePackets(void)	
{
	return (LANLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(lantxQueue));
}
