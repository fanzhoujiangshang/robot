#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "motor_control.h"
#include "led_control.h"
#include "gyro.h"
#include "imu.h"
#include "kalman.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 四轴自稳控制代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

static bool isInit;
setpoint_t		setpoint;	/*设置目标状态*/
sensorData_t 	sensorData;	/*传感器数据*/
state_t 		state;		/*四轴姿态*/
control_t 		control;	/*四轴控制参数*/

void stabilizerInit(void)
{
	if(isInit) return;
	
//	stateControlInit();		/*姿态PID初始化*/
	motorControlInit();		/*电机初始化*/
	ledControlInit();		/*led初始化*/
	imuInit();				/*姿态解算初始化*/
	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= motorControlTest();
	pass &= ledControlTest();	

	return pass;
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	
	//等待陀螺仪校准完成
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	while(1) 
	{
		//1KHz运行频率
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		//获取传感器数据
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);				
		}
		
		//四元数和欧拉角计算
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);		
//		  IMU_GetYawPitchRoll(&state) ;	
			height_control();		
//		auto_yaw_control();
		}
		
		//控制电机输出（500Hz）
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			motorControl(&control);
		}
		//控制led 输出（500Hz）
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			ledControl();
		}
		tick++;
	}
}

