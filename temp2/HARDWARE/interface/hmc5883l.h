#ifndef HMC5883L_H
#define HMC5883L_H

#include "sys.h"
#include "sensors_types.h"
#include <stdbool.h>
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * HMC5883L驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/
bool hmc5883lRead(Axis3i16* magRaw);
bool hmc5883lInit(void);

#endif

//------------------End of File----------------------------
