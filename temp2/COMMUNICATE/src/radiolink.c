#include <string.h>
#include "config.h"
#include "radiolink.h"
#include "config_param.h"
#include "uart_syslink.h"

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

#define RADIOLINK_TX_QUEUE_SIZE  30 /*���ն��и���*/

static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;
static bool isInit;
static atkp_t txPacket;
static atkp_t rxPacket;
static xQueueHandle  radiotxQueue;
static xQueueHandle  radioackQueue;

static void atkpPacketDispatch(atkp_t *rxPacket);

//radiolink����ATKPPacket����
void radiolinkRxTask(void *param)
{
	rxState = waitForStartByte1;
	
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;

	while(1)
	{
		if (uartslkGetDataWithTimout(&c))
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
						atkpPacketDispatch(&rxPacket);
					} 
					else	/*У�����*/
					{
						printf("c = %02x, cksum = %02x",c,cksum);
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
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


void radiolinkInit(void)
{
	if (isInit) return;
	uartslkInit();
	
	/*�������Ͷ��У�CRTP_TX_QUEUE_SIZE����Ϣ*/
	radiotxQueue = xQueueCreate(RADIOLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(radiotxQueue);

	radioackQueue = xQueueCreate(RADIOLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(radioackQueue);
	
	isInit = true;
}

/*���ATKPPacket����ͨ������DMA����*/
static void uartSendPacket(atkp_t *p)
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
	
	/*����DMA����*/
	uartslkSendDataDmaBlocking(dataSize, sendBuffer);
}

// radiolink����ATKPPacket����
void radiolinkTxTask(void *param)
{
	atkp_t p;
	u8 sendBuffer[64];
	u8 cksum;
	u8 dataLen;
	while(1)
	{
		xQueueReceive(radiotxQueue, &p, portMAX_DELAY);
		
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
		uartslkSendDataDmaBlocking(dataLen, sendBuffer);
		//ledseqRun(DATA_TX_LED, seq_linkup);
	}
}

/*radiolink���յ�ATKPPacketԤ����*/
static void atkpPacketDispatch(atkp_t *rxPacket)
{
	atkpReceivePacketBlocking(rxPacket, 	RADIO_TYPE);
	vTaskDelay(50);

	/*���յ�һ��ң���������ݰ�����һ����*/
	if(xQueueReceive(radioackQueue, &txPacket, 0) == pdTRUE)
	{
		ASSERT(txPacket.dataLen <= ATKP_MAX_DATA_SIZE);
//		ledseqRun(DATA_TX_LED, seq_linkup);
		printf("uartSendPacket......\n");

		uartSendPacket(&txPacket);
	} else {
		printf("radioackQueue is null \n");
	}

}

bool radiolinkSendPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(radiotxQueue, p, 0);
}

bool radiolinkSendPacketBlocking(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(radiotxQueue, p, portMAX_DELAY);	
}

bool radiolinkAckPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(radioackQueue, p, 0);
}

//��ȡʣ�����txQueue����
int radiolinkGetFreeTxQueuePackets(void)	
{
	return (RADIOLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(radiotxQueue));
}
