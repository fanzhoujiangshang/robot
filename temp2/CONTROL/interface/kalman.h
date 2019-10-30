#ifndef __KALMEN_H
#define __KALMEN_H


void height_control(void);
void auto_yaw_control(void);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_computeAngles(void);
void Mahony_Init(float sampleFrequency);
struct pid_para
{
	float kp;
	float ki;
	float kd;

	float _last_input;
	float _last_derivative;

	float _integrator;

	float _imax;

	float p_out;
	float i_out;
	float d_out;
};

#define	constrain(amt,low,high)	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
