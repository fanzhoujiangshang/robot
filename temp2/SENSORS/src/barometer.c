#include "barometer.h"
#include "MS5837.h"

#include "i2c.h"

/********************************************************************************	 
 * 深度驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/
static bool isInit = false;
bool baroInit(void)
{
//	return drv_pressure_meas_ms5837_init();
	
	drv_pressure_meas_ms5837_init();
	 isInit = true;
   	 return isInit;
}

u8 i2c_data[9];
void baroUpdate(pressure_data_t *baro)
{
	drv_pressure_meas_ms5837_read(baro);
	
//	Sensors_I2C_Read_Register(0x0B,0x0D,4,i2c_data);
}

