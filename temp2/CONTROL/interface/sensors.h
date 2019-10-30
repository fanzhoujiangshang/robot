#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"
#include "config.h"

/********************************************************************************	 
 * ���������ƴ���	
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/

/*��������ȡ����Ƶ��*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ
#define MAG_UPDATE_RATE			RATE_100_HZ
#define BARO_UPDATE_RATE		RATE_50_HZ

extern sensorData_t sensors;

void sensorsTask(void *param);
void sensorsInit(void);			/*��������ʼ��*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*��ȡ����������*/

bool sensorsIsMagPresent(void);

/* ������������������ */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(pressure_data_t *baro);

extern void IMU_Init(void);
extern void IMU_GetYawPitchRoll(state_t *state); 

#endif //__SENSORS_H
