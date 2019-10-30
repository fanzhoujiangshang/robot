#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "atkp.h"
#include "radiolink.h"
#include "usblink.h"
#include "lanlink.h"
#include "usbd_usr.h"
#include "tcp_server.h" 
#include "stabilizer.h"
#include "motors.h"
#include "pm.h"
#include "sensors.h"
#include "config_param.h"
#include "motor_control.h"
#include "led_control.h"
#include "axis.h"
#include "imu.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * 无线通信驱动代码	
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
 * 说明：此文件程序基于于匿名科创地面站V4.34通信协议下位机
 *     示例代码修改。
********************************************************************************/

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//????????(??ms)
#define  PERIOD_STATUS		0
#define  PERIOD_STATUS1 		1

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

#define  DEVICE_LOCK				0x01
#define  DEEP_LOCK					0x02

uint16_t motor_enable;
uint16_t floating_flag;
uint16_t speed_level;
uint16_t robot_control;

control_radio_t control_radio;
extern control_parameter_t control_parameter;

typedef struct
{
	float roll;			//横滚角
	float pitch;			//俯仰角
	float yaw;			//偏航角
	float depth;			//深度
	float heading;		//相对位置
	float ObstacleDistance;//障碍物距离
	u16 IO_switch;	        //位0  ：设备锁定    位1  ：深度锁定  （1：开启，0：关闭）
}status;

typedef struct
{
	u8 masterslave;  	//主从设备
	float temp;			//温度
	float DeviceBatt; 		//设备电量  
	float BuoyBatt ; 		//浮标电量 
}status_1;

typedef struct	
{
	uint8_t statusTemp;
	uint8_t statusDepth;
	uint8_t statusAngle;
}status_2;

typedef struct
{
	u16 Hardware;
	u16 Software;
}version_info;

bool isInit = false;
static xQueueHandle rxQueue;
static u8 receiveType;
static u16 IOSwitch = 0;
static void atkpSendPacket(atkp_t *p)
{
	radiolinkSendPacket(p);
	
	if(getusbConnectState())
	{
		usblinkSendPacket(p);
	}	
	if(get_tcpclient_connected_status())
	{
		lanlinkSendPacket(p);
	}
}

static void atkpAckPacket(atkp_t *p)
{	
	if(receiveType == RADIO_TYPE)
	{
		radiolinkAckPacket(p);
	}
	if(receiveType == USB_TYPE)
	{
		usblinkAckPacket(p);
	}
	if(receiveType == LAN_TYPE)
	{
		lanlinkAckPacket(p);
	}	
	
}

/***************************??????????******************************/
static void sendStatus(status status )
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(status.roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(status.pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(status.yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	 _temp = (int)(status.depth*100);	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	 _temp = (int)(status.heading*100);	
	p.data[_cnt++] = BYTE1(_temp);
	p.data[_cnt++] = BYTE0(_temp);
	 _temp = status.IO_switch;	
	p.data[_cnt++] = BYTE1(_temp);
	p.data[_cnt++] = BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendStatus1(status_1 status)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	p.msgID = UP_STATUS1;
	
	p.data[_cnt++]=status.masterslave;
	_temp = (int)(status.temp*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	 _temp = (int)(status.DeviceBatt*100);	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	 _temp = (int)(status.BuoyBatt*100);	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendStatus2(status_2 status)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	p.msgID = 0x05; 
	
	p.data[_cnt++]= status.statusTemp;
	p.data[_cnt++]= status.statusDepth;
	p.data[_cnt++]= status.statusAngle;
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendVersionInformation(version_info version)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_VERSION;
	
	_temp = (int)(version.Hardware*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	 _temp = (int)(version.Software*100);	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	atkpAckPacket(&p);
}


static void sendCheck(u8 head, u8 check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	atkpAckPacket(&p);
}
/****************************************************************************/
	status_2 _status_2;

/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
	static u16 count_ms = 0;
	status status;
	status_1 status_1;

	switch(count_ms%2)
	{
		case PERIOD_STATUS:
		{
			status.roll = state.attitude.roll;
			status.pitch = -state.attitude.pitch;
			status.yaw = -state.attitude.yaw;
			status.depth = sensorData.baro.depth;
			status.heading = 10;
			status.ObstacleDistance = 100;
			status.IO_switch = IOSwitch;
//printf("roll=%6.2f ,pitch=%6.2f ,yaw=%6.2f \n",state.attitude.roll,state.attitude.pitch,state.attitude.yaw);			
//printf("depth=%6.2f ,pressure=%6.2f  \n",sensorData.baro.depth, sensorData.baro.pressure);		
			sendStatus(status);	
		}
		break;
										
		case PERIOD_STATUS1:
		{   
			status_1.masterslave = 0xFF;
			status_1.temp = sensorData.baro.temperature;			
			status_1.DeviceBatt = battery_percent;
			status_1.BuoyBatt =   battery_percent;	
//printf("DeviceBatt =%6.2f \n",status_1.DeviceBatt);			
//printf("temperature=%6.2f \n",sensorData.baro.temperature);			
			sendStatus1(status_1);	
		}
		break;
				
	}
	if(count_ms++>=2) 		
		count_ms = 0;	
	
}

static u8 atkpCheckSum(atkp_t *packet)
{
	u8 sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

static void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== DOWN_COMMAND)
	{
		switch(anlPacket->data[0])
		{
			case D_COMMAND_VERSION_INFO:
			{
				version_info version;
				version.Hardware = 10;
				version.Software = configParam.version;
				sendVersionInformation(version);
				break;
			}
			
			case D_COMMAND_DEFAULT_PARAMETER:
				configParamRestoreFactory();				
				break;
			
			case D_COMMAND_ACC_CALIB:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);				
				break;
			
			case D_COMMAND_GYRO_CALIB:
				gyroSetCalibrationCycles(CALIBRATING_ACC_CYCLES);				
				break;
			
			case D_COMMAND_MAG_CALIB:
				compassSetCalibrationStart();				
				break;
		}
		if(anlPacket->data[0] != D_COMMAND_VERSION_INFO)
		{
			u8 cksum = atkpCheckSum(anlPacket);
			sendCheck(anlPacket->msgID,cksum);	
		}

	}			
	else if(anlPacket->msgID == DOWN_COMMAND1)
	{	
		switch(anlPacket->data[0])
		{
			case D_COMMAND1_DEVICE_LOCK:
				IOSwitch |= DEVICE_LOCK;
				break;
			
			case D_COMMAND1_DEVICE_UNLOCK:
				IOSwitch &= (~DEVICE_LOCK);				
				break;
			
			case D_COMMAND1_DEEP_LOCK:
				IOSwitch |= DEEP_LOCK;				
				break;
			
			case D_COMMAND1_DEEP_UNLOCK:
				IOSwitch &= (~DEEP_LOCK);						
				break;
			
		}
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);		
	}
	else if(anlPacket->msgID == DOWN_LED_BRIGHTNESS)
	{
		s16 m1_set = ((s16)(*(anlPacket->data+0)));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	       printf("<led set m1_set=%d>\n",m1_set);		
		setLedPWM(m1_set);	
				
	}
	else if(anlPacket->msgID == DOWN_MOTO_SPEED)	
	{
		control_updown up_down;	

		up_down.updown_x = ((*(anlPacket->data+0)));
		up_down.updown_y = ((*(anlPacket->data+1)));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
//	       printf("<updown_x =%d , updown_y=%d>\n",up_down.updown_x,up_down.updown_y);		
		controlUpDown(up_down);	
	}
	else if(anlPacket->msgID == DOWN_UPDOWN_REMOTE)	
	{
		control_updown up_down;	
//		up_down.updown_x = ((*(anlPacket->data+0)));
//		up_down.updown_y = ((*(anlPacket->data+1)));
		//add..
		control_radio.yaw      =  10*((*(anlPacket->data+0)));
		control_radio.gas    =  10*((*(anlPacket->data+1)));
		control_radio.uwTick1 = uwTick;
//		control_radio.roll     =  ((*(anlPacket->data+2)));
//		control_radio.yaw      =  ((*(anlPacket->data+3)));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	  printf("<updown_x =%d , updown_y=%d>\n",up_down.updown_x,up_down.updown_y);		
//		controlUpDown(up_down);	
	}	
	else if(anlPacket->msgID == DOWN_DIRECTION_REMOTE)	
	{
		control_direction direction;	

//		direction.direction_x= ((*(anlPacket->data+0)));
//		direction.direction_y= ((*(anlPacket->data+1)));
//		control_radio.yaw = 10*((*(anlPacket->data+0)));
		control_radio.lateral = 10*((*(anlPacket->data+0)));
		control_radio.forward = 10*((*(anlPacket->data+1)));
		
		control_radio.uwTick2 = uwTick;
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	       printf("<direction_x =%d , direction_y=%d>\n",direction.direction_x,direction.direction_y);		
//		controlDirection(direction);	
	}	
	else if(anlPacket->msgID == DEVICE_LOCK)
	{
		motor_enable = *(anlPacket->data+0);
	}
	else if(anlPacket->msgID == ENABLE_FOLOAT)
	{
		floating_flag = *(anlPacket->data+0);
	}
	else if(anlPacket->msgID == SPEED_SET)
	{
		speed_level = *(anlPacket->data+0);
		switch(speed_level)
		{
			case 1:
					control_parameter.forward = 0.5f;
					control_parameter.lateral = 0.5f;
					control_parameter.yaw     = 0.3f;
					control_parameter.gas     = 1.0f;
				break;
			case 2:
					control_parameter.forward = 0.75f;
					control_parameter.lateral = 0.75f;
					control_parameter.yaw     = 0.3f;
					control_parameter.gas     = 1.0f;
				break;
			case 3:
					control_parameter.forward = 1.0f;
					control_parameter.lateral = 1.0f;
					control_parameter.yaw     = 0.3f;
					control_parameter.gas     = 1.0f;
				break;
			default:
				break;
		}
	}
	else if(anlPacket->msgID == DEVICE_SET)
	{
		robot_control = *(anlPacket->data+0);
	}
	else if(anlPacket->msgID == GET_SENSOR_INFO)
	{
		sendStatus2(_status_2);
	}
} 

void atkpTxTask(void *param)
{
	while(1)
	{
		atkpSendPeriod();
		vTaskDelay(500);
//		vTaskDelay(500);
	}
}

void atkpRxAnlTask(void *param)
{
	atkp_t p;
	while(1)
	{
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
		atkpReceiveAnl(&p);
	}
}

void atkpInit(void)
{
	if(isInit) return;
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p, u8 type)
{
	receiveType = type;
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
