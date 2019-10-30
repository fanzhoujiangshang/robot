#ifndef __ATKP_H
#define __ATKP_H
#include "sys.h"
#include <stdbool.h>
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 飞控通讯协议格式代码
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
 * 说明：本格式部分基于匿名上位机通讯协议编写
********************************************************************************/

/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30

/*通讯数据结构*/
typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;

/*上行指令ID*/
typedef enum 
{
	UP_VERSION		= 0x00,
	UP_STATUS		= 0x01,
	UP_STATUS1		= 0x02,
	UP_ERROR		= 0x03,	
	UP_CHECK		= 0x04,
	UP_PRINTF	       = 0x05,	
}upmsgID_e;


/*下行指令*/
#define  D_COMMAND_VERSION_INFO		              0x00
#define  D_COMMAND_DEFAULT_PARAMETER		0x01
#define  D_COMMAND_ACC_CALIB					0x02
#define  D_COMMAND_GYRO_CALIB					0x03
#define  D_COMMAND_MAG_CALIB					0x04
#define  D_COMMAND_SELF_TEST					0x05


#define  D_COMMAND1_DEVICE_LOCK				0x00
#define  D_COMMAND1_DEVICE_UNLOCK			0x01
#define  D_COMMAND1_DEEP_LOCK					0x02
#define  D_COMMAND1_DEEP_UNLOCK				0x03



/*下行指令ID*/
typedef enum 
{
	DOWN_COMMAND	  			= 0x00,
	DOWN_COMMAND1			= 0x01,
	DOWN_LED_BRIGHTNESS		= 0x02,
	DOWN_MOTO_SPEED		 	= 0x03,
	DOWN_UPDOWN_REMOTE		= 0x04,
	DOWN_DIRECTION_REMOTE	= 0x05,	
	DEVICE_LOCK  = 0x06,
	ENABLE_FOLOAT = 0x07,
	SPEED_SET = 0x08,
	DEVICE_SET = 0x09,
	UPLOAD_QUERY = 0x10,
	DATA_DOWNLOAD = 0x11,
	GET_SENSOR_INFO = 0x12,
}downmsgID_e;

typedef enum 
{
	RADIO_TYPE	= 0x00,
	USB_TYPE	= 0x01,
	LAN_TYPE	= 0x02,	
}RECEIVE_TYPE;

void atkpTxTask(void *param);
void atkpRxAnlTask(void *param);
void atkpInit(void);
bool atkpReceivePacketBlocking(atkp_t *p, u8 type);

extern float battery_value;
extern float battery_percent;
#endif /*ATKP_H*/

