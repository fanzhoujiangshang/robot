#ifndef __ADC1_H
#define __ADC1_H
#include "sys.h" 

/********************************************************************************	 
 * ATKflight�ɿع̼�
 * adc1��������	
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/

//extern volatile uint16_t adcValues;
extern volatile uint16_t adcValues[2];
void adc1Init(void);

#endif


