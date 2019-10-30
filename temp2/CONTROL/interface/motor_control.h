#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

typedef struct 
{
	u16 m1;
	u16 m2;
	u16 m3;
	u16 m4;
	u16 m5;
	u16 m6;	
	u16 m7;
	u16 m8;		
}motorPWM_t;

typedef struct 
{
	u8 updown_x;
	u8 updown_y;	
}control_updown;

typedef struct
{
	u16 gas;
	u16 pitch;
	u16 roll;
	u16 yaw;
	u16 forward;
	u16 lateral;
	
	u8 button1;
	u8 button2;
	u8 button3;
	u8 button4;
	
	u32 uwTick1;
	u32 uwTick2;
}control_radio_t;


typedef struct
{
	float forward;
	float lateral;
	float yaw;
	float gas;
}control_parameter_t;

#define PWM_MOTOR1 1
#define PWM_MOTOR2 2
#define PWM_MOTOR3 3
#define PWM_MOTOR4 4
#define PWM_MOTOR5 5
#define PWM_MOTOR6 6 
#define PWM_MOTOR7 7
#define PWM_MOTOR8 8

typedef struct 
{
	u8 direction_x;
	u8 direction_y;	
}control_direction;

typedef struct{
	uint16_t ch1, ch2, ch3, ch4, ch5 ,ch6, ch7, ch8;
}pwm_t;

void motorControlInit(void);
bool motorControlTest(void);
void motorControl(control_t *control);

void controlUpDown(control_updown up_down);
void controlDirection(control_direction direction);

void getMotorPWM(motorPWM_t* get);
void setMotorPWM(bool enable, u8 m1_set, u8 m2_set, u8 m3_set, u8 m4_set, u8 m5_set, u8 m6_set, u8 m7_set, u8 m8_set);

void send_pwm_cmd(void *handle, uint16_t pwm[4]);

#endif 
