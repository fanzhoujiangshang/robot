/**
  ******************************************************************************
  * @file    IMU.h
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file contains all the functions prototypes for the IMU firmware 
  *          library.

  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */



#ifndef __IMU_H
#define __IMU_H
#include "sys.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stabilizer_types.h"

#define M_PI  (float)3.1415926535
#define IMU_PRES_TYPE_BM180     1
#define IMU_PRES_TYPE_BM280     2

extern int16_t accel[3], gyro[3];
extern float angles[3];
extern uint8_t u8PressureType;

extern void IMU_Init(void); 
extern void IMU_GetYawPitchRoll(state_t *state);

#endif

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

