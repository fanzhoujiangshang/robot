#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "sensors.h"
#include "stabilizer.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "barometer.h"
#include "sensorsalignment.h"
#include "imu.h"
#include "stm32f4xx_flash.h"
#include "stmflash.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/********************************************************************************	 
 * 传感器控制代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

sensorData_t sensors;

static bool isInit = false;

static bool isMPUPresent=false;
static bool isMagPresent=false;
static bool isBaroPresent=false;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;


/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(pressure_data_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

/* 传感器器件初始化 */
void sensorsInit(void)
{
	
	IMU_Init();
	if (isInit) return;
	
	initBoardAlignment();

	isMPUPresent = gyroInit(GYRO_UPDATE_RATE);
	isMPUPresent = accInit(ACC_UPDATE_RATE);
	isMagPresent  = compassInit();
	isBaroPresent = baroInit();
	
	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(pressure_data_t));
	
	isInit = true;
}

//电池管理
typedef struct
{
	int magx_min;
	int magx_max;
	int magy_min;
	int magy_max;
	int magz_min;
	int magz_max;
}_mag_related_data;

_mag_related_data mag_related_data;


uint8_t mag_calibration_flag;
extern int16_t test_mag_data[3];
//int magx_min,magx_max,magy_min,magy_max,magz_min,magz_max;
int mag_mid[3];
int mag_average[6];

void mag_calibration(void)
{
	static uint32_t i;
	if(mag_calibration_flag)
	{
		if(i == 0)
		{
			mag_related_data.magx_min = mag_related_data.magx_max = test_mag_data[0];
			mag_related_data.magy_min = mag_related_data.magy_max = test_mag_data[1];
			mag_related_data.magz_min = mag_related_data.magz_max = test_mag_data[2];
		}
		
		if(i < 10000)
		{
			i++;
			if(test_mag_data[0] < mag_related_data.magx_min)
			{
				mag_related_data.magx_min = test_mag_data[0];
			}
			if(test_mag_data[0] > mag_related_data.magx_max)
			{
				mag_related_data.magx_max = test_mag_data[0];
			}
			
			if(test_mag_data[1] < mag_related_data.magy_min)
			{
				mag_related_data.magy_min = test_mag_data[1];
			}
			if(test_mag_data[1] > mag_related_data.magy_max)
			{
				mag_related_data.magy_max = test_mag_data[1];
			}
			
			if(test_mag_data[2] < mag_related_data.magz_min)
			{
				mag_related_data.magz_min = test_mag_data[2];
			}
			if(test_mag_data[2] > mag_related_data.magz_max)
			{
				mag_related_data.magz_max = test_mag_data[2];
			}
			
			mag_mid[0] = (mag_related_data.magx_min + mag_related_data.magx_max)/2;
			mag_mid[1] = (mag_related_data.magy_min + mag_related_data.magy_max)/2;
			mag_mid[2] = (mag_related_data.magz_min + mag_related_data.magz_max)/2;
		}
	}
	
	if(i == 10000)
	{
		mag_calibration_flag = false;
		i = 0;
		STMFLASH_Write(0X080E0000,(u32*)&mag_related_data,6);
		delay_ms(2000);
		STMFLASH_Read(0X080E0000,(u32*)mag_average,6);
	}
	
}


/*传感器任务*/
void sensorsTask(void *param)
{	
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	sensorsInit();		//传感器初始化
	
	while (1)
	{
//	printf("sensorsTask \n");		
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz运行频率

		if (isMPUPresent && RATE_DO_EXECUTE(GYRO_UPDATE_RATE, tick))
		{
			gyroUpdate(&sensors.gyro);
//			IMU_GetYawPitchRoll(&state) ;
		}
		
		if (isMPUPresent && RATE_DO_EXECUTE(ACC_UPDATE_RATE, tick))
		{
			accUpdate(&sensors.acc);
		}
	
		if (isMagPresent && RATE_DO_EXECUTE(MAG_UPDATE_RATE, tick))
		{
			compassUpdate(&sensors.mag);
		}
		
		if (isBaroPresent && RATE_DO_EXECUTE(BARO_UPDATE_RATE, tick))
		{
			baroUpdate(&sensors.baro);
		}
		
		mag_calibration();
		
		vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
		xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
		xQueueOverwrite(gyroDataQueue, &sensors.gyro);
		if (isMagPresent)
		{
			xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
		}
		if (isBaroPresent){
			xQueueOverwrite(barometerDataQueue, &sensors.baro);
	       }
		xTaskResumeAll();

		tick++;
	}	
}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
	sensorsReadBaro(&sensors->baro);
	
	
//	printf("gyro_x=%.1f ,gyro_y=%.1f ,gyro_z=%.1f \n",sensors->gyro.x,sensors->gyro.y,sensors->gyro.z);	
//	printf("acc_x=%.1f ,acc_y=%.1f ,acc_z=%.1f \n",sensors->acc.x,sensors->acc.y,sensors->acc.z);	
//	printf("pressure=%.1f ,depth=%.1f ,temperature=%.1f \n",sensors->baro.pressure,sensors->baro.depth,sensors->baro.temperature);	
}

bool sensorsIsMagPresent(void)
{
	return isMagPresent;
}
