#include "kalman.h"
#include "sensors.h"
#include "motor_control.h"
#include "stabilizer.h"

#include "math.h"

extern sensorData_t sensors;
extern control_radio_t control_radio;

float target_height,height_error;
char auto_height_key;

float target_yaw,yaw_error;
char auto_yaw_key;

struct pid_para alt_height_pid;
struct pid_para yaw_pid;

float auto_gas,auto_pid_gas;
uint32_t auto_height_gas_count;
float auto_yaw,auto_pid_yaw;
uint32_t auto_yaw_count;

signed int Get_Pid(struct pid_para *pid_t, signed short err, float dtime)
{
	signed int p_part,i_part,d_part;
	float _derivative;

	//比例部分
	p_part = err * pid_t->kp;
	pid_t->p_out = p_part;
	//积分部分
	if((fabs(pid_t->ki)>1e-7) && (fabs(dtime)>1e-7))
	{
		pid_t->_integrator += ((float)err * pid_t->ki)*dtime;
		pid_t->_integrator = constrain(pid_t->_integrator, -pid_t->_imax, pid_t->_imax);
		i_part = pid_t->_integrator;		
	}
	else
	{
		i_part = 0;
	}
	pid_t->i_out = i_part;
	//微分部分
	if((fabs(pid_t->kd)>1e-7) && (fabs(dtime)>1e-7))
	{
		_derivative = ((float)(err - pid_t->_last_input))/dtime;
		_derivative = (_derivative + pid_t->_last_derivative)*0.5f;
		pid_t->_last_input = err;
		pid_t->_last_derivative = _derivative;
		d_part = (signed short)(pid_t->kd * _derivative);	
	}
	else
	{
		d_part = 0;
	}
	pid_t->d_out = d_part;
	return (p_part + i_part + d_part);

}

void auto_yaw_control(void)
{
	
	yaw_pid.kp = 2.3;  //0.5
	yaw_pid.ki = 1.0; //0.01
	yaw_pid.kd = 0.1;  //0.1
	yaw_pid._imax = 200.0;
	
	if(sensors.baro.depth >= 50)
	{
		if((control_radio.pitch>=450)&&(control_radio.pitch<=550))
		{
			auto_yaw_count++;
			if(auto_yaw_count == 250)
			{
				auto_yaw_key = true;
				target_yaw = state.attitude.yaw;
			}
		}
		else
		{
			auto_height_gas_count = 0;
			auto_height_key = false;
			auto_yaw = 0;
		}
		
		if(auto_yaw_key)
		{
			yaw_error = target_yaw - state.attitude.yaw;
			auto_pid_yaw = Get_Pid(&yaw_pid, yaw_error, 0.005);
			auto_pid_yaw = constrain(auto_pid_yaw,-500,500);
			auto_yaw = 500 + auto_pid_yaw;
		}
		
	}
	else
	{
		auto_height_gas_count = 0;
		auto_height_key = false;
		auto_yaw		= 0;
	}
}	

//height control  height loop
void height_control(void)
{
	alt_height_pid.kp = 0.3;
	alt_height_pid.ki = 0.01; 
	alt_height_pid.kd = 0.01;
	alt_height_pid._imax = 250.0;
	
	if(sensors.baro.depth >= 50)
	{
		
		if((control_radio.gas>=450)&&(control_radio.gas<=550))
		{
			auto_height_gas_count++;
			if(auto_height_gas_count == 500)
			{
				auto_height_key = true;
				target_height = sensors.baro.depth;
			}
		}
		else
		{
			auto_height_gas_count = 0;
			auto_height_key = false;
			auto_gas = 0;
		}
		if(auto_height_key)
		{
			height_error = target_height - sensors.baro.depth;
			auto_pid_gas = Get_Pid(&alt_height_pid, height_error, 0.005);
			auto_pid_gas = constrain(auto_pid_gas,-500,500);
			auto_gas = 500 + auto_pid_gas;
		}
	}
	else
	{
		auto_height_gas_count = 0;
		auto_height_key = false;
		auto_gas = 0;
	}	
}

static float q0,q1,q2,q3;
float integralFBx, integralFBy, integralFBz; 
float invSampleFreq;
float twoKi;
char anglesComputed;
float roll, pitch, yaw;

#define twoKpDef	(120.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(0.5f * 1.0f)	// 2 * integral gain

void Mahony_Init(float sampleFrequency)
{
	twoKi = twoKiDef;	// 2 * integral gain (Ki)
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	anglesComputed = 0;
	invSampleFreq = 1.0f / sampleFrequency;
}

float Mahony_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

extern int16_t test_mag_data[3];
float Mx,My;
void Mahony_computeAngles()
{
	roll = 57.29578f*atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = 57.29578f*asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = 57.29578f*atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
//	My = ((float)test_mag_data[1])*cos(roll) + ((float)(test_mag_data[1]))*sin(roll)*sin(pitch) - ((float)test_mag_data[2])*sin(roll)*cos(pitch);
//	Mx = ((float)test_mag_data[0])*cos(pitch) + ((float)test_mag_data[2])*sin(pitch);
//	yaw = atan(My/Mx)*57.3f;
	anglesComputed = 1;
}
float getRoll() {
	if (!anglesComputed) Mahony_computeAngles();
	return roll * 57.29578f;
}
float getPitch() {
	if (!anglesComputed) Mahony_computeAngles();
	return pitch * 57.29578f;
}
float getYaw() {
	if (!anglesComputed) Mahony_computeAngles();
	return yaw * 57.29578f + 180.0f;
}




