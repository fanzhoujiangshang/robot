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
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

#define LANLINK_TX_QUEUE_SIZE  30 /*���ն��и���*/
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

//radiolink����ATKPPacket����
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
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,���ݳ���Ϊ0��У��1*/
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
					if (cksum == c)	/*����У����ȷ*/
					{		
						atkpReceivePacketBlocking(&rxPacket, 	LAN_TYPE);		
						vTaskDelay(50);
						/*���յ�һ�����ݰ�����һ����*/
						if(xQueueReceive(lanackQueue, &txPacket, 0) == pdTRUE)
						{
							ASSERT(txPacket.dataLen <= ATKP_MAX_DATA_SIZE);
//							printf("lanSendPacket......\n");							

							lanSendPacket(&txPacket);
						}else{
//							printf("lanSendPacketis null \n");
						}								
					} 
					else	/*У�����*/
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
		else	/*��ʱ����*/
		{
			rxState = waitForStartByte1;
		}	
	}
}


void lanlinkInit(void)
{
	if (isInit) return;
	
	 tcpserverInit();	
	 
	/*�������Ͷ��У�CRTP_TX_QUEUE_SIZE����Ϣ*/
	lantxQueue = xQueueCreate(LANLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(lantxQueue);

	lanackQueue = xQueueCreate(LANLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(lanackQueue);
	
	isInit = true;
}

/*���ATKPPacket���ݷ���*/
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
	dataSize = p->dataLen + 5;//����cksum
	/*����У���*/
	for (int i=0; i<dataSize-1; i++)
	{
		cksum += sendBuffer[i];
	}
	sendBuffer[dataSize-1] = cksum;
	
	TcpServerSendDataBlocking(dataSize, sendBuffer);
}

// radiolink����ATKPPacket����
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

//��ȡʣ�����txQueue����
int lanlinkGetFreeTxQueuePackets(void)	
{
	return (LANLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(lantxQueue));
}
