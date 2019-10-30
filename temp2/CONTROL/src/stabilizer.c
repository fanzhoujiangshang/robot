#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "motor_control.h"
#include "led_control.h"
#include "gyro.h"
#include "imu.h"
#include "kalman.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * �������ȿ��ƴ���	
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/

static bool isInit;
setpoint_t		setpoint;	/*����Ŀ��״̬*/
sensorData_t 	sensorData;	/*����������*/
state_t 		state;		/*������̬*/
control_t 		control;	/*������Ʋ���*/

void stabilizerInit(void)
{
	if(isInit) return;
	
//	stateControlInit();		/*��̬PID��ʼ��*/
	motorControlInit();		/*�����ʼ��*/
	ledControlInit();		/*led��ʼ��*/
	imuInit();				/*��̬�����ʼ��*/
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
	
	//�ȴ�������У׼���
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	while(1) 
	{
		//1KHz����Ƶ��
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		//��ȡ����������
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);				
		}
		
		//��Ԫ����ŷ���Ǽ���
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);		
//		  IMU_GetYawPitchRoll(&state) ;	
			height_control();		
//		auto_yaw_control();
		}
		
		//���Ƶ�������500Hz��
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			motorControl(&control);
		}
		//����led �����500Hz��
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			ledControl();
		}
		tick++;
	}
}

