#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * ��������:2018/6/22
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) 
 * All rights reserved
********************************************************************************/


#define MOTOR_M1  		1
#define MOTOR_M2  		2
#define MOTOR_M3  		3
#define MOTOR_M4  		4
#define MOTOR_M5  		5
#define MOTOR_M6  		6
#define MOTOR_M7  		7
#define MOTOR_M8  		8

#define MOTORS_PWM_RATIO         214/200	//20%
#define MOTORS_TEST_RATIO         1500
#define MOTORS_TEST_ON_TIME_MS    10
#define MOTORS_TEST_DELAY_TIME_MS 1500

void motorsInit(void);		/*�����ʼ��*/
bool motorsTest(void);		/*�������*/
void motorsSetRatio(u32 id, u16 ithrust);	/*���õ��ռ�ձ�*/
void motor_power_enable(void);
void motor_power_disable(void);

#endif /* __MOTORS_H */

