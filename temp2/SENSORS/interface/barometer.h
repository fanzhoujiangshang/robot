#ifndef __BAROMETER_H
#define __BAROMETER_H

#include "sys.h"
#include "stabilizer_types.h"
/********************************************************************************	 
 * 气压计驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/
bool baroInit(void);
void baroUpdate(pressure_data_t *baro);

#endif

