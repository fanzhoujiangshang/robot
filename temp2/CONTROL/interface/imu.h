#ifndef __IMU_H
#define __IMU_H

#include "sys.h"
#include "axis.h"
#include "maths.h"
#include "stabilizer_types.h"

/********************************************************************************	 
 * ��̬������������	
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/

extern float imuAttitudeYaw;


void imuInit(void);
void imuTransformVectorBodyToEarth(Axis3f * v);
void imuTransformVectorEarthToBody(Axis3f * v);
void imuUpdateAttitude(const sensorData_t *sensorData, state_t *state, float dt);

#endif


