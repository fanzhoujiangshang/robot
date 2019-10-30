#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
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

void motorsInit(void);		/*电机初始化*/
bool motorsTest(void);		/*电机测试*/
void motorsSetRatio(u32 id, u16 ithrust);	/*设置电机占空比*/
void motor_power_enable(void);
void motor_power_disable(void);

#endif /* __MOTORS_H */

