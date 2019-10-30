#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"
#include "FreeRTOS.h"		 
#include "task.h"

/********************************************************************************	 
 * 四轴自稳控制代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

#define MAIN_LOOP_RATE 			RATE_500_HZ				//主循环速率
#define MAIN_LOOP_DT			(1.0/MAIN_LOOP_RATE)	

#define ATTITUDE_ESTIMAT_RATE	RATE_100_HZ				//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define POSITION_ESTIMAT_RATE	ATTITUDE_ESTIMAT_RATE	//位置预估速率（和姿态解算速率一致）
#define POSITION_ESTIMAT_DT		(1.0/POSITION_ESTIMAT_RATE)

#define RATE_PID_RATE			MAIN_LOOP_RATE 			//角速度环PID速率（和主循环速率一致）
#define RATE_PID_DT				(1.0/RATE_PID_RATE)

#define ANGLE_PID_RATE			ATTITUDE_ESTIMAT_RATE 	//角度环PID速率（和姿态解算速率一致）
#define ANGLE_PID_DT			(1.0/ANGLE_PID_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE 	//位置速度环PID速率（和位置预估速率一致）
#define VELOCITY_PID_DT			(1.0/VELOCITY_PID_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE 	//位置环PID速率（和位置预估速率一致）
#define POSITION_PID_DT			(1.0/POSITION_PID_RATE)

extern setpoint_t		setpoint;	/*设置目标状态*/
extern sensorData_t 	sensorData;	/*传感器数据*/
extern state_t 			state;		/*四轴姿态*/
extern control_t 		control;	/*四轴控制参数*/

void stabilizerInit(void);
bool stabilizerTest(void);
void stabilizerTask(void* param);

#define uwTick      (xTaskGetTickCount())

#endif /* __STABALIZER_H */
