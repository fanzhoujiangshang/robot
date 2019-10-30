#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "config.h"
#include "config_param.h"
#include "watchdog.h"
#include "stmflash.h"
#include "delay.h"

/*FreeRTOS相关头文件*/
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


#define VERSION 11	/*12 表示V1.2*/

configParam_t configParam;

static configParam_t configParamDefault=
{
	.version = VERSION,		/*软件版本号*/

	.accBias= 	/*加速度校准值*/
	{
		.accZero = 
		{
			0,
			0,
			0,
		},
		.accGain = 
		{
			4096,
			4096,
			4096,
		}
	},
	.magBias=
	{
		.magZero = 
		{
			0,
			0,
			0,
		},
	},
	.boardAlign=
	{
		.rollDeciDegrees = 0,
		.pitchDeciDegrees = 0,
		.yawDeciDegrees = 0,
	},	
};

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;

static SemaphoreHandle_t  xSemaphore = NULL;


static u8 configParamCksum(configParam_t* data)
{
	int i;
	u8 cksum=0;	
	u8* c = (u8*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	
	return cksum;
}

void configParamInit(void)	/*参数配置初始化*/
{
	if(isInit) return;
	
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth);
	
	if(configParam.version == VERSION)	/*版本正确*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*校验正确*/
		{
			printf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		} else
		{
			printf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}	
	else	/*版本更新*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*配置参数错误，写入默认参数*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*计算校验值*/
		STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*写入stm32 flash*/
		isConfigParamOK=true;
	}	
	
	xSemaphore = xSemaphoreCreateBinary();
	
	isInit=true;
}

void configParamRestoreFactory(void)	/*恢复默认参数配置*/
{
	memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
	configParam.cksum = configParamCksum(&configParam);				/*计算校验值*/
	STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*写入stm32 flash*/
	isConfigParamOK=true;
	

}

void configParamTask(void* param)
{
	u8 cksum = 0;
	
	while(1) 
	{	
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cksum = configParamCksum(&configParam);		/*数据校验*/
		
		if(configParam.cksum != cksum)	
		{
			configParam.cksum = cksum;	/*数据校验*/
			watchdogInit(250);			/*擦除时间比较长，看门狗时间设置大一些*/  //250					
			STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*写入stm32 flash*/
			watchdogInit(WATCHDOG_RESET_MS);		/*重新设置看门狗*/
		}						
	}
}

bool configParamTest(void)
{
	return isInit;
}

void configParamGiveSemaphore(void)
{
	xSemaphoreGive(xSemaphore);		
}



void saveConfigAndNotify(void)
{
	u8 cksum = configParamCksum(&configParam);		/*数据校验*/
	if(configParam.cksum != cksum)	
	{
		configParam.cksum = cksum;	/*数据校验*/				
		STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*写入stm32 flash*/
	}
}
