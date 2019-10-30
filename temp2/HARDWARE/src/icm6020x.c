#include "icm2060x.h"
#include "spi1.h"
#include "delay.h"
#include "stdio.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * ICM2060X驱动代码	
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

#define NULL 0

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1


#define ACC_GYRO_RAWDATA_LEN	14

#define DISABLE_ICM2060X()	GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define ENABLE_ICM2060X()   	GPIO_ResetBits(GPIOA, GPIO_Pin_15);


static bool isInit = false;

bool icm2060XSpiWriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_ICM2060X();
	delay_ms(1);
	spi1TransferByte(reg);
	spi1TransferByte(data);
    DISABLE_ICM2060X();
	delay_ms(1);
	return true;
}

bool icm2060XSpiReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
	ENABLE_ICM2060X();
	spi1TransferByte(reg | 0x80); // read transaction
    spi1Transfer(data, NULL, length);
	DISABLE_ICM2060X();
	return true;
}

bool icm2060XInit(void)
{
	if (isInit) return true;
	
	//SPI1初始化
	spi1Init();
	spi1SetSpeed(SPI_CLOCK_SLOW);
	
	GPIO_InitTypeDef GPIO_InitStructure;

	//配置ICM2060X(PC2)引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//复位ICM2060X
	icm2060XSpiWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	delay_ms(50);
	icm2060XSpiWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	delay_ms(50);
	icm2060XSpiWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);//复位两次增加传感器稳定性
	delay_ms(50);
	icm2060XSpiWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	delay_ms(50);
	
	//读取ID
	u8 id = 0x00;
	icm2060XSpiReadRegister(MPU_RA_WHO_AM_I, 1, &id);
	
	printf("id = %d \n",id);	
	
	//读取正常，初始化
	if(id == ICM2060X_WHO_AM_I_CONST)
	{
		//设置X轴陀螺作为时钟 
		icm2060XSpiWriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX);
		delay_ms(15);
		
		//禁止I2C接口
		icm2060XSpiWriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
		delay_ms(15);
		icm2060XSpiWriteRegister(MPU_RA_PWR_MGMT_2, 0x00);
		delay_ms(15);
		
		// Accel Sample Rate 1kHz
		// Gyroscope Output Rate =  1kHz when the DLPF is enabled
		icm2060XSpiWriteRegister(MPU_RA_SMPLRT_DIV, 0);//设置采样率
		delay_ms(15);
		
		//设置陀螺仪 +/- 2000 DPS量程
		icm2060XSpiWriteRegister(MPU_RA_GYRO_CONFIG, FSR_2000DPS << 3);
		delay_ms(15);
		
		//设置加速度 +/- 8 G 量程
		icm2060XSpiWriteRegister(MPU_RA_ACCEL_CONFIG, FSR_8G << 3);
		delay_ms(15);
		
		//设置中断引脚功能
		icm2060XSpiWriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);//中断引脚配置
		delay_ms(15);
		
		//设置低通滤波带宽
		icm2060XSpiWriteRegister(MPU_RA_CONFIG, BITS_DLPF_CFG_98HZ);
		delay_ms(1);
		
		u8 buffer;
		icm2060XSpiReadRegister(MPU_RA_PWR_MGMT_1, 1, &buffer);		
		printf("MPU_RA_CONFIG = %d \n",buffer);			
		
		printf("ICM2060X SPI connection [OK].\n");
		isInit = true;
	}
	else
	{
		printf("ICM2060X SPI connection [FAIL].\n");
	}
	
    spi1SetSpeed(SPI_CLOCK_STANDARD);//设置SPI为中速模式

	return isInit;
}

bool icm2060XGyroRead(Axis3i16* gyroRaw)
{
	if(!isInit) 
		return false;
	u8 buffer[6];
	icm2060XSpiReadRegister(MPU_RA_GYRO_XOUT_H, 6, buffer);
	gyroRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
	gyroRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
	gyroRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];

	printf("x=%d ,y=%d ,z=%d \n",gyroRaw->x,gyroRaw->y,gyroRaw->z);		
	return true;
}

bool icm2060XAccRead(Axis3i16* accRaw)
{
	if(!isInit) 
		return false;
	u8 buffer[6];
	icm2060XSpiReadRegister(MPU_RA_ACCEL_XOUT_H, 6, buffer);
	accRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
	accRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
	accRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];

	printf("x1=%d ,y1=%d ,z1=%d \n",accRaw->x,accRaw->y,accRaw->z);	
	return true;
}

