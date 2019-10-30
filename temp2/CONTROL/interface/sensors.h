#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"
#include "config.h"

/********************************************************************************	 
 * 传感器控制代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

/*传感器读取更新频率*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ
#define MAG_UPDATE_RATE			RATE_100_HZ
#define BARO_UPDATE_RATE		RATE_50_HZ

extern sensorData_t sensors;

void sensorsTask(void *param);
void sensorsInit(void);			/*传感器初始化*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*获取传感器数据*/

bool sensorsIsMagPresent(void);

/* 单独测量传感器数据 */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(pressure_data_t *baro);

extern void IMU_Init(void);
extern void IMU_GetYawPitchRoll(state_t *state); 

#endif //__SENSORS_H
