#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "axis.h"
#include "maths.h"
#include "imu.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "kalman.h"

/********************************************************************************	 
 * 姿态解算驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

/**	
 * 姿态解算规则如下：
 *     ROLL  = 绕X轴旋转，右手定则，逆时针为正顺时针为负。
 *     PITCH = 绕Y轴旋转，右手定则，逆时针为正顺时针为负。
 *     YAW   = 绕Z轴旋转，右手定则，逆时针为正顺时针为负。
 */

#define DCM_KP_ACC			0.600f		//加速度补偿陀螺仪PI参数
#define DCM_KI_ACC			0.005f

#define DCM_KP_MAG			1.000f		//磁力计补偿陀螺仪PI参数
#define DCM_KI_MAG			0.000f

#define IMU_SMALL_ANGLE		15.0f		//满足水平状态的最小角度（单位deg）

#define SPIN_RATE_LIMIT     20			//旋转速率

extern int16_t test_mag_data[3];

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//四元数
static float rMat[3][3];//四元数的旋转矩阵		
//static float smallAngleCosZ;//水平最小角余弦值

static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuInit(void)
{
//	smallAngleCosZ = cos_approx(degreesToRadians(IMU_SMALL_ANGLE));//最小倾角余弦值
    cos_approx(degreesToRadians(IMU_SMALL_ANGLE));//最小倾角余弦值
    imuComputeRotationMatrix();
	
		Mahony_Init(150);
}

void imuTransformVectorBodyToEarth(Axis3f *v)
{
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = -y;//
    v->z = z;
}

void imuTransformVectorEarthToBody(Axis3f *v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = y;
    v->z = z;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//磁力计快速增益
static float imuMagFastPGainSaleFactor(void)
{
	//四轴上电后磁力计融合需要一段时间
	//为了快速融合，前100次使用快速增益
	static u32 magFastPGainCount = 100;
	
//	if (!ARMING_FLAG(ARMED) && (magFastPGainCount--))
//		return 10.0f;
//	else
	if(magFastPGainCount--)
		return 10.0f;
	else	
		return 1.0f;
}

static void imuMahonyAHRSupdate(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz,
								bool useMag,float dt)
{
	static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    //加速度积分误差
	static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    //磁力计积分误差
	float ex, ey, ez;

    //计算旋转速率(rad/s)
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    //Step 1: Yaw correction
    if (useMag) 
	{
		const float magMagnitudeSq = mx * mx + my * my + mz * mz;
		float kpMag = DCM_KP_MAG * imuMagFastPGainSaleFactor();
		
		if (magMagnitudeSq > 0.01f) 
		{
			//单位化磁力计测量值
			const float magRecipNorm = invSqrt(magMagnitudeSq);
			mx *= magRecipNorm;
			my *= magRecipNorm;
			mz *= magRecipNorm;
		
			//计算X\Y方向的磁通，磁北方向磁通
			const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
			const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
			const float bx = sqrtf(hx * hx + hy * hy);

			//磁力计误差是估计磁北和测量磁北之间的交叉乘积
			const float ez_ef = -(hy * bx);

			//旋转误差到机体坐标系
			ex = rMat[2][0] * ez_ef;
			ey = rMat[2][1] * ez_ef;
			ez = rMat[2][2] * ez_ef;
		}
		else 
		{
			ex = 0;
			ey = 0;
			ez = 0;
		}

		//累计误差补偿
		if (DCM_KI_MAG > 0.0f) 
		{
			//如果旋转速率大于限制值则停止积分
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) 
			{
				integralMagX += DCM_KI_MAG * ex * dt;
				integralMagY += DCM_KI_MAG * ey * dt;
				integralMagZ += DCM_KI_MAG * ez * dt;

				gx += integralMagX;
				gy += integralMagY;
				gz += integralMagZ;
			}
		}
		
		//误差补偿
		gx += kpMag * ex;
		gy += kpMag * ey;
		gz += kpMag * ez;
	}

	
    //Step 2: Roll and pitch correction
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		//单位化加速计测量值
		const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= accRecipNorm;
		ay *= accRecipNorm;
		az *= accRecipNorm;

		//加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
		ex = (ay * rMat[2][2] - az * rMat[2][1]);
		ey = (az * rMat[2][0] - ax * rMat[2][2]);
		ez = (ax * rMat[2][1] - ay * rMat[2][0]);

		//累计误差补偿
		if (DCM_KI_ACC > 0.0f) 
		{
			//如果旋转速率大于限制值则停止积分
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
			{
				integralAccX += DCM_KI_ACC * ex * dt;
				integralAccY += DCM_KI_ACC * ey * dt;
				integralAccZ += DCM_KI_ACC * ez * dt;

				gx += integralAccX;
				gy += integralAccY;
				gz += integralAccZ;
			}
		}

		//误差补偿
		gx += DCM_KP_ACC * ex;
		gy += DCM_KP_ACC * ey;
		gz += DCM_KP_ACC * ez;
	}
	
	//一阶近似算法，四元数运动学方程的离散化形式和积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

	//单位化四元数 
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    //计算四元数的旋转矩阵
    imuComputeRotationMatrix();
}


float imuAttitudeYaw;//范围：-180~180，用于上传到匿名上位机（支持范围-180~180）

float tmp_sine_p,tmp_sine_r,tmp_cosine_p,tmp_cosine_r;
float navig_megx,navig_megy;
float meg_azimuth;
float meg_azimuth_out;
float MEG_OFFSET=-1.920*0.01745329252;

//更新欧拉角
static void imuUpdateEulerAngles(attitude_t *attitude)
{

	attitude->pitch = -RADIANS_TO_DEGREES(atan2_approx(rMat[2][1], -rMat[2][2]));
	attitude->roll = RADIANS_TO_DEGREES((0.5f * M_PIf) - acos_approx(-rMat[2][0]));//arcsin = 0.5PI - arccos
//	attitude->yaw = RADIANS_TO_DEGREES(atan2_approx(rMat[1][0], rMat[0][0])) + 90;
	imuAttitudeYaw  = RADIANS_TO_DEGREES(atan2_approx(rMat[1][0], rMat[0][0])) + 90;
	if(imuAttitudeYaw > 360)
	{
		imuAttitudeYaw -= 360;
	}
	attitude->yaw = imuAttitudeYaw;
//	imuAttitudeYaw = attitude->yaw;
	
//	tmp_sine_p = sin(DEGREES_TO_RADIANS(attitude->pitch));
//	tmp_cosine_p = cos(DEGREES_TO_RADIANS(attitude->pitch));
//	tmp_sine_r = sin(DEGREES_TO_RADIANS(attitude->roll));
//	tmp_cosine_r = cos(DEGREES_TO_RADIANS(attitude->roll));
//	
//	navig_megx = -test_mag_data[1]*tmp_cosine_p + test_mag_data[0]*tmp_sine_p*tmp_sine_r + test_mag_data[2]*tmp_cosine_r*tmp_sine_p;//
//	navig_megy = test_mag_data[0]*tmp_cosine_r + test_mag_data[2]*tmp_sine_r;	//
//	
//	if(navig_megx == 0.0)
//	{
//		if(navig_megy<0.0)						                                                                                                                                                
//			meg_azimuth = 1.57079633;
//		else
//			meg_azimuth = 4.71238898;
//	}
//	else if(navig_megx < 0.0)
//	{
//		meg_azimuth = 3.141592653 - atan(navig_megy/navig_megx);		
//	}
//	else
//	{
//		if(navig_megy < 0.0)
//			meg_azimuth = -atan(navig_megy/navig_megx);	
//		else
//			meg_azimuth = 6.28318531 - atan(navig_megy/navig_megx);
//	}
//	meg_azimuth += MEG_OFFSET;
//	if(meg_azimuth>6.28318531)
//		meg_azimuth-=6.28318531;

//	
//	meg_azimuth = atan(navig_megy/navig_megx);
//	meg_azimuth_out = meg_azimuth * 57.3f;
	
//	meg_azimuth = atan2(test_mag_data[1],test_mag_data[0]);
//	meg_azimuth_out = meg_azimuth * 57.3f;
	


	if (attitude->yaw < 0.0f)//转换位0~360
		attitude->yaw += 360.0f;

	//更新最小倾角状态
//	if (rMat[2][2] > smallAngleCosZ) 
//		ENABLE_STATE(SMALL_ANGLE);
//	else 
//		DISABLE_STATE(SMALL_ANGLE);
}
extern int mag_calibration_data[9];
int related_magx,related_magy,related_magz;

void imuUpdateAttitude(const sensorData_t *sensorData, state_t *state, float dt)
{
	bool useMag = compassIsHealthy();
	
	Axis3f gyro = sensorData->gyro;
	Axis3f acc  = sensorData->acc;
	Axis3f mag  = sensorData->mag;
	
	//角速度单位由度转为弧度
	gyro.x = (gyro.x * DEG2RAD)/32.8f;
	gyro.y = (gyro.y * DEG2RAD)/32.8f;
	gyro.z = (gyro.z * DEG2RAD)/32.8f;
	

	related_magx = test_mag_data[0] - mag_calibration_data[6];
	related_magy = test_mag_data[1] - mag_calibration_data[7];
	related_magz = test_mag_data[2] - mag_calibration_data[8];
	//计算四元数和旋转矩阵
    imuMahonyAHRSupdate(gyro.x, gyro.y, gyro.z,
                        acc.x, acc.y, acc.z,
                        related_magx, related_magy, related_magz,
						useMag,dt);
	
    //计算欧拉角               
    imuUpdateEulerAngles(&state->attitude);

//	Mahony_update(gyro.x, gyro.y, gyro.z,
//								acc.x, acc.y, acc.z,
//								related_magx, related_magy, related_magz);
//	
//	Mahony_computeAngles();
	
}







