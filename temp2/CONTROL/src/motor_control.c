#include "motor_control.h"
#include "motors.h"
#include "math.h"
#include "stabilizer.h"
#include "sensors.h"
/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet={0, 0, 0, 0, 0, 0, 0, 0};
extern control_radio_t control_radio;
control_parameter_t control_parameter;
extern uint16_t motor_enable;
extern char auto_height_key;
extern float auto_gas;
extern char auto_yaw_key;
extern float auto_yaw;


void ControlParameterInit(void)
{
	control_parameter.forward = 0.5f;
	control_parameter.lateral = 0.5f;
	control_parameter.yaw     = 0.15f;
	control_parameter.gas     = 0.5f;
	
	motor_enable = true;
}

void motorControlInit(void)
{
	motorsInit();
	ControlParameterInit();
	
}

bool motorControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

u16 limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

uint8_t test_flag;

void send_pwm_cmd(void *handle, uint16_t pwm[4]){
	uint16_t sum;
	sum = pwm[0] + pwm[1] + pwm[2] + pwm[3];
	printf("pwm: %hd %hd %hd %hd %hd\n", pwm[0], pwm[1], pwm[2], pwm[3], sum);
}


#define LIMIT_PWM_DOWN(X) (X < 1000 ? 1000 : X)
#define LIMIT_PWM_UP(X) (X > 2000 ? 2000 : X)

uint16_t pwm_channels[9];
uint8_t motor_number;
uint16_t pwm_value = 1500;

void motorControl(control_t *control)	/*功率输出控制*/
{
	u8 i;
	
	if(motor_enable)
	{
		
//		if(sensors.baro.altitude <= -50)
//		{
//			if(auto_height_key)
//			{
//				 control_radio.gas = auto_gas;
//			} 	
//		}
		
		
//motorsSetRatio(motor_test_number, motor_test_value);
//                        Motor #      Roll Pitch Yaw  Throttle Forward Lateral Testing Order	
//	add_motor_raw_6dof(AP_MOTORS_MOT_1, 0,    0,  1.0f, 0,     -1.0f,    1.0f,           1);
//  add_motor_raw_6dof(AP_MOTORS_MOT_2, 0,    0, -1.0f, 0,     -1.0f,   -1.0f,           2);
//  add_motor_raw_6dof(AP_MOTORS_MOT_3, 0,    0, -1.0f, 0,      1.0f,    1.0f,           3);
//  add_motor_raw_6dof(AP_MOTORS_MOT_4, 0,    0,  1.0f, 0,      1.0f,   -1.0f,           4);
//  add_motor_raw_6dof(AP_MOTORS_MOT_5, 1.0f, 0,  0,   -1.0f,   0,       0,              5);
//  add_motor_raw_6dof(AP_MOTORS_MOT_6,-1.0f, 0,  0,   -1.0f,   0,       0,              6);
//  反桨	顺桨       具体转向根据电机实际安装方向和三相电路情况
//   2     1         pwm信号都小于1500  机器人向前和向上运动
//	 6     5        
//   4     3                           
//                                    前后杆量 前负后正                航向杆量 左负右正     			 左右偏移杆量 左负右正

			if(auto_yaw_key)
			{
				pwm_channels[PWM_MOTOR1] = 1500 - control_parameter.forward*(500 - control_radio.forward ) - 0.81f*control_parameter.yaw*(500 - auto_yaw) - 0.81f*control_parameter.lateral*(500 - control_radio.lateral);
				pwm_channels[PWM_MOTOR2] = 1500 - control_parameter.forward*(500 - control_radio.forward ) + 0.81f*control_parameter.yaw*(500 - auto_yaw) + 0.81f*control_parameter.lateral*(500 - control_radio.lateral);
				pwm_channels[PWM_MOTOR3] = 1500 - control_parameter.forward*(500 - control_radio.forward ) - control_parameter.yaw*(500 - auto_yaw) + 		  control_parameter.lateral*(500	- control_radio.lateral);
				pwm_channels[PWM_MOTOR4] = 1500 - control_parameter.forward*(500 - control_radio.forward ) + control_parameter.yaw*(500 - auto_yaw) - 		  control_parameter.lateral*(500	- control_radio.lateral);
			}
			else 
			{
				pwm_channels[PWM_MOTOR1] = 1500 - control_parameter.forward*(500 - control_radio.forward ) - 0.81f*control_parameter.yaw*(500 - control_radio.yaw) - 0.81f*control_parameter.lateral*(500 - control_radio.lateral);
				pwm_channels[PWM_MOTOR2] = 1500 - control_parameter.forward*(500 - control_radio.forward ) + 0.81f*control_parameter.yaw*(500 - control_radio.yaw) + 0.81f*control_parameter.lateral*(500 - control_radio.lateral);
				pwm_channels[PWM_MOTOR3] = 1500 - control_parameter.forward*(500 - control_radio.forward ) - control_parameter.yaw*(500 - control_radio.yaw) +       control_parameter.lateral*(500	- control_radio.lateral);
				pwm_channels[PWM_MOTOR4] = 1500 - control_parameter.forward*(500 - control_radio.forward ) + control_parameter.yaw*(500 - control_radio.yaw) -       control_parameter.lateral*(500	- control_radio.lateral);
			}          
//			上下		
			if(auto_height_key)
			{
				pwm_channels[PWM_MOTOR5] = 1500 - control_parameter.gas*(500 - auto_gas);
				pwm_channels[PWM_MOTOR6] = 1500 - control_parameter.gas*(500 - auto_gas);
			}
			else 
			{
				pwm_channels[PWM_MOTOR5] = 1500 - control_parameter.gas*(500 - control_radio.gas);
				pwm_channels[PWM_MOTOR6] = 1500 - control_parameter.gas*(500 - control_radio.gas);
			}
//			motorsSetRatio(motor_number,pwm_value);
			
			for(i=1;i<=6;i++)
			{
				motorsSetRatio(i,pwm_channels[i]);
			}
			 
			if(uwTick - control_radio.uwTick1 >= 250)
			{
				control_radio.gas = 500;
			}		
			if(uwTick - control_radio.uwTick2 >= 250)
			{
				control_radio.yaw     = 500;
				control_radio.lateral = 500;
				control_radio.forward = 500;
			}
		}
		else  //motor lock
		{
				pwm_channels[PWM_MOTOR1] = 1500;
				pwm_channels[PWM_MOTOR2] = 1500;
				pwm_channels[PWM_MOTOR3] = 1500;
				pwm_channels[PWM_MOTOR4] = 1500;
				
				pwm_channels[PWM_MOTOR5] = 1500;
				pwm_channels[PWM_MOTOR6] = 1500;
				for(i=1;i<=6;i++)
				{
					motorsSetRatio(i,pwm_channels[i]);
				}
		}
	
	
#if 0
	s16 r = control->roll / 2.0f;
	s16 p = control->pitch / 2.0f;
	
	motorPWM.m1 = limitThrust(control->thrust - r - p + control->yaw);
	motorPWM.m2 = limitThrust(control->thrust - r + p - control->yaw);
	motorPWM.m3 = limitThrust(control->thrust + r + p + control->yaw);
	motorPWM.m4 = limitThrust(control->thrust + r - p - control->yaw);		

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*控制电机输出百分比*/
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
#endif
	
	
	
//	if (motorSetEnable)
//	{
//		motorPWM = motorPWMSet;
//		motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*控制电机输出百分比*/
//		motorsSetRatio(MOTOR_M2, motorPWM.m2);
//		motorsSetRatio(MOTOR_M3, motorPWM.m3);
//		motorsSetRatio(MOTOR_M4, motorPWM.m4);	
//		motorsSetRatio(MOTOR_M5, motorPWM.m5);
//		motorsSetRatio(MOTOR_M6, motorPWM.m6);	
//		motorsSetRatio(MOTOR_M7, motorPWM.m7);
//		motorsSetRatio(MOTOR_M8, motorPWM.m8);			
//		printf("<powerControl....>\n");
//		
//	} else {
//		motorsSetRatio(MOTOR_M1, MOTORS_TEST_RATIO);	/*控制电机输出百分比*/
//		motorsSetRatio(MOTOR_M2, MOTORS_TEST_RATIO);
//		motorsSetRatio(MOTOR_M3, MOTORS_TEST_RATIO);
//		motorsSetRatio(MOTOR_M4, MOTORS_TEST_RATIO);	
//		motorsSetRatio(MOTOR_M5, MOTORS_TEST_RATIO);
//		motorsSetRatio(MOTOR_M6, MOTORS_TEST_RATIO);	
//		motorsSetRatio(MOTOR_M7, MOTORS_TEST_RATIO);
//		motorsSetRatio(MOTOR_M8, MOTORS_TEST_RATIO);	
////		printf("<motorControl....>\n");
//	}

}

void controlUpDown(control_updown up_down)
{
	motorSetEnable = 1;
//	motorPWMSet.m1 = up_down.updown_x;
#if 0
	motorPWMSet.m4 = MOTORS_TEST_RATIO + (15-up_down.updown_y)*100*MOTORS_PWM_RATIO;
	motorPWMSet.m5 = MOTORS_TEST_RATIO - (15-up_down.updown_y)*100*MOTORS_PWM_RATIO;
	motorPWMSet.m7 = MOTORS_TEST_RATIO - (15-up_down.updown_y)*100*MOTORS_PWM_RATIO;;	
	 printf("<motorPWMSet.m4 =%d , motorPWMSet.m7=%d>\n",motorPWMSet.m4,motorPWMSet.m7);	
#endif
//	motorPWMSet.m4 = up_down.updown_y*100;
//	motorPWMSet.m5 = up_down.updown_y*100;
	motorPWMSet.m7 = up_down.updown_y*100;
	 printf("<motorPWMSet.m4 =%d , motorPWMSet.m7=%d>\n",motorPWMSet.m4,motorPWMSet.m7);
//	motorsSetRatio(MOTOR_M1, motorPWMSet.m1);	/*控制电机输出百分比*/
//	motorsSetRatio(MOTOR_M4, motorPWMSet.m4);	
//	motorsSetRatio(MOTOR_M5, motorPWMSet.m5);	
	motorsSetRatio(MOTOR_M7, motorPWMSet.m7);	
	motorsSetRatio(MOTOR_M1, motorPWMSet.m7);	
}
void controlDirection(control_direction direction)
{
	motorSetEnable = 1;
//	motorPWMSet.m3 = direction.direction_x;
//	motorPWMSet.m4 = direction.direction_y;
	 printf("<motorPWMSet.m3 =%d , motorPWMSet.m4=%d>\n",motorPWMSet.m3,motorPWMSet.m4);	
//	motorsSetRatio(MOTOR_M3, motorPWMSet.m3);	/*控制电机输出百分比*/
//	motorsSetRatio(MOTOR_M4, motorPWMSet.m4);	

}

void getMotorPWM(motorPWM_t* get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u8 m1_set, u8 m2_set, u8 m3_set, u8 m4_set, u8 m5_set, u8 m6_set, u8 m7_set, u8 m8_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;	
	motorPWMSet.m4 = m4_set;
	motorPWMSet.m5 = m5_set;	
	motorPWMSet.m6 = m6_set;
	motorPWMSet.m7 = m7_set;	
	motorPWMSet.m8 = m8_set;	
}
