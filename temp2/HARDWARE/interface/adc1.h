#ifndef __ADC1_H
#define __ADC1_H
#include "sys.h" 

/********************************************************************************	 
 * ATKflight飞控固件
 * adc1驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

//extern volatile uint16_t adcValues;
extern volatile uint16_t adcValues[2];
void adc1Init(void);

#endif


