#ifndef __BAROMETER_H
#define __BAROMETER_H

#include "sys.h"
#include "stabilizer_types.h"
/********************************************************************************	 
 * ��ѹ����������	
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/
bool baroInit(void);
void baroUpdate(pressure_data_t *baro);

#endif

