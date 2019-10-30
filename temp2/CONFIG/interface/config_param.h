#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
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
	u8 version;						/*����汾��*/
	accBias_t accBias;				/*���ٶ�У׼ֵ*/
	magBias_t magBias;				/*������У׼ֵ*/
	boardAlignment_t boardAlign;	/*����΢��*/	
	u8 cksum;			
} configParam_t;


extern configParam_t configParam;

void configParamInit(void);	/*�������ó�ʼ��*/
void configParamRestoreFactory(void);
void configParamTask(void* param);	/*������������*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void saveConfigAndNotify(void);

#endif /*__CONFIG_PARAM_H */

