#include "system.h"	/*头文件集合*/

/********************************************************************************	 
 * main.c	
 * 包括系统初始化和创建任务
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

TaskHandle_t startTaskHandle;

static void startTask(void *arg);

int main() 
{
	systemInit();			/*底层硬件初始化*/	
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/

	vTaskStartScheduler();	/*开启任务调度*/

	while(1){}; 
}
/*创建任务*/
void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*进入临界区*/

	xTaskCreate(tcpServerTask, "TCPSERVER", 150, NULL, 7, NULL);	
	
	xTaskCreate(lanlinkRxTask, "LANLINK_RX", 150, NULL, 6, NULL);	 
	xTaskCreate(lanlinkTxTask, "LANLINK_TX", 150, NULL, 6, NULL);	
	
	xTaskCreate(radiolinkRxTask, "RADIOLINK_RX", 150, NULL, 10, NULL);		/*创建串口连接任务*/
	xTaskCreate(radiolinkTxTask, "RADIOLINK_TX", 150, NULL, 3, NULL);		/*创建串口发送任务*/
	
	
	xTaskCreate(usblinkRxTask, "USBLINK_RX", 150, NULL, 4, NULL);		/*创建usb接收任务*/
	xTaskCreate(usblinkTxTask, "USBLINK_TX", 150, NULL, 3, NULL);		/*创建usb发送任务*/
	
	xTaskCreate(atkpTxTask, "ATKP_TX", 150, NULL, 3, NULL);				/*创建atkp发送任务任务*/
	xTaskCreate(atkpRxAnlTask, "ATKP_RX_ANL", 300, NULL, 6, NULL);		/*创建atkp解析任务*/
	
//	xTaskCreate(configParamTask, "CONFIG_TASK", 150, NULL, 1, NULL);	/*创建参数配置任务*/
	
	xTaskCreate(pmTask, "PWRMGNT", 150, NULL, 2, NULL);					/*创建电源管理任务*/

	xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);			/*创建传感器处理任务*/
	
	xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);		/*创建姿态任务*/
	
	printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/
	
	vTaskDelete(startTaskHandle);										/*删除开始任务*/
		
	taskEXIT_CRITICAL();	/*退出临界区*/
} 

void vApplicationIdleHook( void )
{
	static u32 tickWatchdogReset = 0;

	portTickType tickCount = getSysTickCnt();

	if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS)
	{
		tickWatchdogReset = tickCount;
		watchdogReset();
	}
	
	__WFI();	/*进入低功耗模式*/
}



