/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mpu9250.h"
#include "spi1.h"
#include "delay.h"
#include "stdio.h"
#include "math.h"

#include "motors.h"

//////////////////////////////////////////////////////////////////////////
//
static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};

int16_t test_accel_data[3];
int16_t test_gyro_data[3];
int16_t test_mag_data[3];
int16_t normal_accel;
//////////////////////////////////////////////////////////////////////////
#if 0
//basic SPI driver for MPU9250
static SPI_Driver mMPU9250 = {
	SPI2, RCC_APB1PeriphClockCmd, RCC_APB1Periph_SPI2,
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB,
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB, GPIO_Pin_12,
	GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15,
	GPIO_PinSource13, GPIO_PinSource14, GPIO_PinSource15,	
#ifdef SPIx_USE_DMA

#endif
	{
		SPI_Direction_2Lines_FullDuplex, SPI_Mode_Master, SPI_DataSize_8b, 
		SPI_CPOL_High, SPI_CPHA_2Edge, SPI_NSS_Soft, SPI_BaudRatePrescaler_32,
		SPI_FirstBit_MSB, 7
	},
	GPIO_AF_SPI2
};
static SPI_Driver* pMPU9250 = &mMPU9250;

//
static EXTI_Driver mMPU9250INT= {
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB, GPIO_Pin_8, 
	EXTI_PortSourceGPIOB, EXTI_PinSource8,
	{
		EXTI_Line8, EXTI_Mode_Interrupt, EXTI_Trigger_Rising, ENABLE
	},
	{
		EXTI9_5_IRQn, 14, 0, ENABLE
	}
};
static EXTI_Driver* pMPU9250INT = &mMPU9250INT;
#endif
//////////////////////////////////////////////////////////////////////////
//
//#define MPU9250_SPIx_SendByte(byte) SPIx_SendByte(pMPU9250, byte);
//#define MPU9250_SPIx_SetDivisor(divisor) SPIx_SetDivisor(pMPU9250, divisor);
#define MPU9250_SPIx_SendByte(byte) spi1TransferByte( byte);
#define DISABLE_MPU9250()	GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define ENABLE_MPU9250()   	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
//////////////////////////////////////////////////////////////////////////
//init

static u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u16 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
		{
		retry++;
		if(retry>2000)
			return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
		{
		retry++;
		if(retry>2000)
			return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //接收数据					    
}

uint8_t res;

u8 MPU9250_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	ENABLE_MPU9250()   //	MPU9250_CS=0;  //片选MPU9250
	delay_ms(1);
	status=SPI1_ReadWriteByte(reg); //发送reg地址
	SPI1_ReadWriteByte(value);//发送数据
	DISABLE_MPU9250();//	MPU9250_CS=1;  //失能MPU9250
	return(status);//
}

u8 MPU9250_Read_Reg(u8 reg)
{
	  u8 reg_val;
		ENABLE_MPU9250();//	MPU9250_CS=0;  //片选MPU9250
	  SPI1_ReadWriteByte(reg|0x80); //reg地址+读命令
	  reg_val=SPI1_ReadWriteByte(0xff);//任意数据
		DISABLE_MPU9250();//	MPU9250_CS=1;  //失能MPU9250
	return(reg_val);
}

static void i2c_Mag_write(u8 reg,u8 value)
{
	u16 j=10000;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR);//设置磁力计地址,mode: write
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	j = 10000;
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);//set reg addr
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	j = 10000;
	MPU9250_Write_Reg(I2C_SLV0_DO ,value);//send value	
	while(j--);//此处因为MPU内部I2C读取速度较慢，延时等待内部写完毕
}

static u8 i2c_Mag_read(u8 reg)
{
	u16 j=10000;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR|0x80); //设置磁力计地址，mode：read
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	j = 10000;
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);// set reg addr
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	j = 10000;
	MPU9250_Write_Reg(I2C_SLV0_DO ,0xff);//read
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	return MPU9250_Read_Reg(EXT_SENS_DATA_00);
}

uint8_t id = 0x00;

u8 hmc_add[8];

uint8_t MPU92_ReadReg( uint8_t readAddr )
{
  uint8_t readData;

  ENABLE_MPU9250();
  spi1TransferByte(0x80 | readAddr);
  readData = spi1TransferByte(0x00);
  DISABLE_MPU9250();

  return readData;
}

void MPU92_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  ENABLE_MPU9250();
  spi1TransferByte(writeAddr);
  spi1TransferByte(writeData);
  DISABLE_MPU9250();
}

uint8_t MPU92_AUX_ReadReg( uint8_t slaveAddr, uint8_t readAddr )
{
  uint8_t status;
  uint8_t readData;
  uint32_t timeout = 50000;

  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_ADDR, slaveAddr | 0x80);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_REG, readAddr);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);
  do {
    delay_ms(1);
    status = MPU92_ReadReg(MPU9250_I2C_MST_STATUS);
  } while (((status & MPU9250_I2C_SLV4_DONE) == 0) && (timeout--));
  delay_ms(1);
  readData = MPU92_ReadReg(MPU9250_I2C_SLV4_DI);

  return readData;
}

void MPU92_AUX_WriteReg( uint8_t slaveAddr, uint8_t writeAddr, uint8_t writeData )
{
  uint8_t  status;
  uint32_t timeout = 50000;

  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_ADDR, slaveAddr);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_REG, writeAddr);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_DO, writeData);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);
  do {
    delay_ms(1);
    status = MPU92_ReadReg(MPU9250_I2C_MST_STATUS);
  } while (((status & MPU9250_I2C_SLV4_DONE) == 0) && (timeout--));
  delay_ms(1);
}
void MPU92_AUX_AK8963_Init( void )
{
  uint8_t res, asa[3] = {0};

  delay_ms(1);
  MPU92_AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL2, 0x01);    // Reset Device
  delay_ms(50);
  MPU92_AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x00);    // Power-down mode
  delay_ms(1);
  MPU92_AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x1F);    // Fuse ROM access mode, Read sensitivity adjustment
  delay_ms(10);
  asa[0] = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAX);
  delay_ms(1);
  asa[1] = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAY);
  delay_ms(1);
  asa[2] = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_ASAZ);
  delay_ms(1);
  MPU92_AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x00);    // Power-down mode
  delay_ms(10);
  MPU92_AUX_WriteReg(AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);    // Continuous measurement mode 2 & 16-bit
  delay_ms(10);

//  AK8963_ASA[0] = (asa[0] - 128) * 0.5 / 128 + 1;
//  AK8963_ASA[1] = (asa[1] - 128) * 0.5 / 128 + 1;
//  AK8963_ASA[2] = (asa[2] - 128) * 0.5 / 128 + 1;

//  res = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_CNTL1) & 0x10;
//  switch (res) {
//    case 0x00:  mag_sensadj[0] = mag_sensadj[1] = mag_sensadj[2] = 0.6;   break;
//    case 0x10:  mag_sensadj[0] = mag_sensadj[1] = mag_sensadj[2] = 0.15;  break;
//  }

//  mag_sensadj[0] *= AK8963_ASA[0];
//  mag_sensadj[1] *= AK8963_ASA[1];
//  mag_sensadj[2] *= AK8963_ASA[2];

}

void MPU92_AUX_SlaveConfig( uint8_t slaveNum, uint8_t slaveAddr, uint8_t readAddr, uint8_t readLens )
{
  uint8_t reg;
  uint8_t offset = slaveNum * 3;

  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV0_ADDR + offset, slaveAddr | 0x80);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV0_REG + offset, readAddr);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV0_CTRL + offset, MPU9250_I2C_SLVx_EN | readLens);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_SLV4_CTRL, 0x09);
  delay_ms(1);
  reg = MPU92_ReadReg(MPU9250_I2C_MST_DELAY_CTRL);
  delay_ms(1);
  MPU92_WriteReg(MPU9250_I2C_MST_DELAY_CTRL, reg | (0x01 << slaveNum));
}

void MPU9250_Init(void)
{
	u8 data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	u8 i;
	//读取ID
//	uint8_t id = 0x00;

	GPIO_InitTypeDef GPIO_InitStructure;

	//配置MPU9250_CS(PC2)引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//Lower level hardware Init
//	SPIx_Init(pMPU9250);
//	EXTIx_Init(pMPU9250INT);
	spi1Init();
	spi1SetSpeed(SPI_CLOCK_SLOW);
	
//	MPU9250_Write_Reg(MPU9250_PWR_MGMT_1, 0x00);	//解除休眠状态
//	delay_ms(100);
//	id = MPU9250_Read_Reg(MPU9250_WHO_AM_I);
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_CONFIG, 0x07); 
//	delay_ms(1);
	//低通滤波频率，典型值：0x07(3600Hz)此寄存器内决定Internal_Sample_Rate==8K
	
///**********************Init SLV0 i2c**********************************/	
////Use SPI-bus read slave0
//	MPU9250_Write_Reg(MPU9250_INT_PIN_CFG ,0x30);// INT Pin / Bypass Enable Configuration  
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_I2C_MST_CTRL,0x4d);//I2C MAster mode and Speed 400 kHz
//	delay_ms(1);	
//	MPU9250_Write_Reg(MPU9250_USER_CTRL ,0x20); // I2C_MST _EN 
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_I2C_MST_DELAY_CTRL ,0x01);//延时使能I2C_SLV0 _DLY_ enable 	
//	delay_ms(1);
//	MPU9250_Write_Reg(I2C_SLV0_CTRL ,0x81); //enable IIC	and EXT_SENS_DATA==1 Byte
//	delay_ms(1);	
///*******************Init GYRO and ACCEL******************************/	
//	MPU9250_Write_Reg(0x19, 0x07);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
//	delay_ms(1);
//	MPU9250_Write_Reg(MPU9250_ACCEL_CONFIG2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
//	delay_ms(1);		
///**********************Init MAG **********************************/
//	i2c_Mag_write(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
//	delay_ms(10);
//	i2c_Mag_write(AK8963_CNTL1_REG,0x12); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
//	delay_ms(100);
	
	
	//MPU9250 Reset
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay_ms(200);
  id = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_WHO_AM_I);
	res = id;
	printf("id =%d \n",id); 

	if (0x71 == id) {
		//		//MPU9250 Set Clock Source
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
		delay_ms(1);
		//MPU9250 Set Interrupt
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG, MPU9250_INT_ANYRD_2CLEAR);//MPU9250_INT_ANYRD_2CLEAR
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, 1);
		delay_ms(1);
		//MPU9250 Set Sensors
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
		delay_ms(1);
		//MPU9250 Set SampleRate
		//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, 0x07);
		delay_ms(1);
		//MPU9250 Set Full Scale Gyro Range
		//Fchoice_b[1:0] = [00] enable DLPF
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, 0x10);
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, 0);
		delay_ms(1);
		//MPU9250 Set Full Scale Accel Range PS:2G
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, 0x01);//(MPU9250_FSR_2G << 3)
		delay_ms(1);
		//MPU9250 Set Accel DLPF
//		data = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
//		data |= MPU9250_ACCEL_DLPF_41HZ;
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, 0x08);
		delay_ms(1);
		//MPU9250 Set Gyro DLPF
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
		delay_ms(1);
		//MPU9250 Set SPI Mode
		state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
		delay_ms(1);
		state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
		delay_ms(1);
		//////////////////////////////////////////////////////////////////////////
		//AK8963 Setup
		//reset AK8963
//		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
//		delay_ms(1);

//		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
//		delay_ms(1);
//		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
//		delay_ms(1);
//		//
//		//AK8963 get calibration data
//		MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
//		//AK8963_SENSITIVITY_SCALE_FACTOR
//		//AK8963_ASA[i++] = (int16_t)((data - 128.0f) / 256.0f + 1.0f) ;
//		MPU9250_AK8963_ASA[0] = (int16_t)(response[0]) + 128;
//		MPU9250_AK8963_ASA[1] = (int16_t)(response[1]) + 128;
//		MPU9250_AK8963_ASA[2] = (int16_t)(response[2]) + 128;
//		delay_ms(1);
//		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
//		delay_ms(1);
//		//
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);//0x5D
//		delay_ms(1);
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
//		delay_ms(1);
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
//		delay_ms(1);
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
//		delay_ms(1);
//		//
//		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
//		delay_ms(1);
//		//
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);//0x09
//		delay_ms(1);
//		//
//		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);	
		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_RSV, MPU9250_AK8963_CNTL2_SRST);
		delay_ms(1);

		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
		delay_ms(1);
		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
		delay_ms(1);
		
//		hmc_add[0] = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_WIA);
		//
		//AK8963 get calibration data
		MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
		//AK8963_SENSITIVITY_SCALE_FACTOR
		//AK8963_ASA[i++] = (int16_t)((data - 128.0f) / 256.0f + 1.0f) ;
		MPU9250_AK8963_ASA[0] = (int16_t)(response[0]) + 128;
		MPU9250_AK8963_ASA[1] = (int16_t)(response[1]) + 128;
		MPU9250_AK8963_ASA[2] = (int16_t)(response[2]) + 128;
		delay_ms(1);
		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
		delay_ms(1);
		//
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);//0x5D
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
		delay_ms(1);
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
		delay_ms(1);
		//
		MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
		delay_ms(1);
		//
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x01);//0x09
		delay_ms(1);
		//
		MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
		delay_ms(100);	

//		delay_ms(10);
//		MPU92_WriteReg(MPU9250_PWR_MGMT_1,         0x80);   // Reset Device
//		delay_ms(100);
//		MPU92_WriteReg(MPU9250_PWR_MGMT_1,         0x04);   // Clock Source
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_INT_PIN_CFG,        0x02);   // INT_ANYRD_2CLEAR 0x10
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_INT_ENABLE,         0x01);   // Set RAW_RDY_EN
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_PWR_MGMT_2,         0x00);   // Enable Accel & Gyro
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_SMPLRT_DIV,         0x00);   // Sample Rate Divider, INTERNAL_SAMPLE_RATE = 1KHz
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_GYRO_CONFIG,        0x18);   // 0x00:250dps, 0x08:500dps, 0x10:1000dps, 0x18:2000dps
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_ACCEL_CONFIG,       0x08);   // 0x00:2g, 0x08:4g, 0x10:8g, 0x18:16g
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_CONFIG,             0x00);   // gyro low pass filter
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_ACCEL_CONFIG2,     0x05);   // accel low pass filter
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_USER_CTRL,          0x30);   // Set I2C_MST_EN, I2C_IF_DIS
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_I2C_MST_CTRL,       0x5D);   // aux i2c 400kHz
//		delay_ms(1);
//		MPU92_WriteReg(MPU9250_I2C_MST_DELAY_CTRL, 0x80);   // 
//		delay_ms(1);

//		delay_ms(1000);
//    hmc_add[0] = MPU92_AUX_ReadReg(AK8963_I2C_ADDR, AK8963_WIA);

//		MPU92_AUX_AK8963_Init();
//		MPU92_AUX_SlaveConfig(1, AK8963_I2C_ADDR, AK8963_ST1, 8);   // add ak8963 to i2c slave 0
//		delay_ms(5);
		printf("MPU6000 SPI connection [OK].\n");
	} else {
		printf("MPU6000 SPI connection [FAIL].\n");
	}
}

int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
//	Chip_Select(pMPU9250);
	ENABLE_MPU9250();
//	delay_ms(1);
	MPU9250_SPIx_SendByte(reg_addr);
	MPU9250_SPIx_SendByte(data);
//	Chip_DeSelect(pMPU9250);
	DISABLE_MPU9250();
	return 0;
}

int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
//	Chip_Select(pMPU9250);
	ENABLE_MPU9250();
//	delay_ms(1);
	MPU9250_SPIx_SendByte(reg_addr);
	while(i < len){
		MPU9250_SPIx_SendByte(data[i++]);
	}
//	Chip_DeSelect(pMPU9250);
       DISABLE_MPU9250();
	return 0;
}

u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 dummy = 0;
	u8 data = 0;

//	Chip_Select(pMPU9250);
	ENABLE_MPU9250();
//	delay_ms(1);
	MPU9250_SPIx_SendByte(0x80 | reg_addr);
	data = MPU9250_SPIx_SendByte(dummy);
//	Chip_DeSelect(pMPU9250);
       DISABLE_MPU9250();
	return data;
}

int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	u8 dummy = 0x00;

//	Chip_Select(pMPU9250);
	ENABLE_MPU9250();
//	delay_ms(1);
//	delay_ms(50);
	MPU9250_SPIx_SendByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = MPU9250_SPIx_SendByte(dummy);
	}
//	Chip_DeSelect(pMPU9250);
       DISABLE_MPU9250();
	return 0;
}

int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data) {
	u8 status = 0;
	u32 timeout = 0;

	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	delay_ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	delay_ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	delay_ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data){
	u8 index = 0;
	u8 status = 0;
	u32 timeout = 0;
	u8 tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		delay_ms(1);
		index++;
	}
	return 0;
}



u8 test_temp_status;
int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(10);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	delay_ms(10);
	tmp = data;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	delay_ms(10);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	delay_ms(10);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
		
		test_temp_status = status;
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;
	u8 index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////
//
u8 motor_test_number = 0;
u16 motor_test_value = 1500;


void MPU9250_Get9AxisRawData(Axis3i16 *accel, Axis3i16 * gyro, Axis3i16 * mag)
{
	u8 data[22];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 22, data);//22
	
	accel->x = (data[0] << 8) | data[1];
	accel->y = (data[2] << 8) | data[3];
	accel->z = (data[4] << 8) | data[5];
		
	test_accel_data[0] = (data[0] << 8) | data[1];
	test_accel_data[1] = (data[2] << 8) | data[3];
	test_accel_data[2] = (data[4] << 8) | data[5];
	
	gyro->x = (data[8] << 8) | data[9];
	gyro->y = (data[10] << 8) | data[11];
	gyro->z = (data[12] << 8) | data[13];
	
	test_gyro_data[0] = (data[8] << 8) | data[9];
	test_gyro_data[1] = (data[10] << 8) | data[11];
	test_gyro_data[2] = (data[12] << 8) | data[13];
//	printf("gyro-x:%d,gyro-y:%d,gyro-z:%d\n",gyro->x,gyro->y,gyro->z);
	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag->x = (data[16] << 8) | data[15];
	mag->y = (data[18] << 8) | data[17];
	mag->z = (data[20] << 8) | data[19];
	
	test_mag_data[0] = (data[16] << 8) | data[15];
	test_mag_data[1] = (data[18] << 8) | data[17];
	test_mag_data[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag->x = ((long)mag->x * MPU9250_AK8963_ASA[0]) >> 8;
	mag->y = ((long)mag->y * MPU9250_AK8963_ASA[1]) >> 8;
	mag->z = ((long)mag->z * MPU9250_AK8963_ASA[2]) >> 8;
	printf("mag[0]:%d,mag[1]:%d,mag[2]:%d\n",mag->x,mag->y,mag->z);

//		READ_MPU9250_MAG();
//		READ_MPU9250_ACCEL();
//		READ_MPU9250_GYRO();

}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(Axis3i16 *accel, Axis3i16 * gyro)
{
	u8 data[14];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel->x = (data[0] << 8) | data[1];
	accel->y = (data[2] << 8) | data[3];
	accel->z = (data[4] << 8) | data[5];

	gyro->x = (data[8] << 8) | data[9];
	gyro->y = (data[10] << 8) | data[11];
	gyro->z = (data[12] << 8) | data[13];
}
//////////////////////////////////////////////////////////////////////////

void MPU9250_Get3AxisAccelRawData(Axis3i16 * accel)
{
	u8 data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 6, data);

	accel->x = (data[0] << 8) | data[1];
	accel->y = (data[2] << 8) | data[3];
	accel->z = (data[4] << 8) | data[5];
	
	test_accel_data[0] = (data[0] << 8) | data[1];
	test_accel_data[1] = (data[2] << 8) | data[3];
	test_accel_data[2] = (data[4] << 8) | data[5];
	
	normal_accel = sqrt((test_accel_data[0]*test_accel_data[0])+(test_accel_data[1]*test_accel_data[1])
	+(test_accel_data[2]*test_accel_data[2]));
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(Axis3i16 * gyro)
 {
	u8 data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_GYRO_XOUT_H, 6, data);

	gyro->x = (data[0] << 8) | data[1];
	gyro->y = (data[2] << 8) | data[3];
	gyro->z = (data[4] << 8) | data[5];
	
	test_gyro_data[0] = (data[0] << 8) | data[1];
	test_gyro_data[1] = (data[2] << 8) | data[3];
	test_gyro_data[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
void MPU9250_Get3AxisMagnetRawData(Axis3i16 *mag)
{
//	mag_data[0] = i2c_Mag_read(MAG_XOUT_L);
	u8 data[8];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_EXT_SENS_DATA_00, 8, data);
	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[7] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag->x = (data[2] << 8) | data[1];
	mag->y = (data[4] << 8) | data[3];
	mag->z = (data[6] << 8) | data[5];
	
	test_mag_data[0] = (data[2] << 8) | data[1];
	test_mag_data[1] = (data[4] << 8) | data[3];
	test_mag_data[2] = (data[6] << 8) | data[5];

	mag->x = ((long)mag->x * MPU9250_AK8963_ASA[0]) >> 8;
	mag->y = ((long)mag->y * MPU9250_AK8963_ASA[1]) >> 8;
	mag->z = ((long)mag->z * MPU9250_AK8963_ASA[2]) >> 8;
	
	
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(long *temperature)
{
	u8 data[2];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_TEMP_OUT_L, 2, data);
	temperature[0] = (((s16)data[0]) << 8) | data[1];
}
#if 0
static vu8 MPU9250_IsNewData = 0;

int MPU9250_IsDataReady(void)
{
	int isNewData = MPU9250_IsNewData;
	MPU9250_IsNewData = 0;
	return isNewData;
}

//////////////////////////////////////////////////////////////////////////
//
void EXTI9_5_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET){
    EXTI_ClearITPendingBit(EXTI_Line8);
		MPU9250_IsNewData = 1;
  }
}

#endif
