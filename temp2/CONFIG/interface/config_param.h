#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/
												
typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
	
	pidInit_t x;
	pidInit_t y;
	pidInit_t z;
} pidParamPos_t;

typedef struct
{
	int16_t accZero[3];
	int16_t accGain[3];
} accBias_t;

typedef struct
{
	int16_t magZero[3];
} magBias_t;

typedef struct 
{
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
} boardAlignment_t;

typedef struct	
{
	u8 version;						/*软件版本号*/
	accBias_t accBias;				/*加速度校准值*/
	magBias_t magBias;				/*磁力计校准值*/
	boardAlignment_t boardAlign;	/*板子微调*/	
	u8 cksum;			
} configParam_t;


extern configParam_t configParam;

void configParamInit(void);	/*参数配置初始化*/
void configParamRestoreFactory(void);
void configParamTask(void* param);	/*参数配置任务*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void saveConfigAndNotify(void);

#endif /*__CONFIG_PARAM_H */

