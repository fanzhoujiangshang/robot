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

#ifndef _MPU9250_H
#define _MPU9250_H

#include "stm32f4xx.h"
#include "sys.h"
#include "sensors_types.h"
#include <stdbool.h>
//////////////////////////////////////////////////////////////////////////
//Register Map for Gyroscope and Accelerometer
#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19
#define MPU9250_CONFIG                  0x1A
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
//
#define MPU9250_I2C_READ 0x80

#define MPU9250_I2C_SLVx_EN         ((uint8_t)0x80)
#define MPU9250_I2C_SLV4_DONE       ((uint8_t)0x40)
#define MPU9250_I2C_SLV4_NACK       ((uint8_t)0x10)


//Magnetometer register maps
#define MPU9250_AK8963_WIA                 0x00
#define MPU9250_AK8963_INFO                0x01
#define MPU9250_AK8963_ST1                 0x02
#define MPU9250_AK8963_XOUT_L              0x03
#define MPU9250_AK8963_XOUT_H              0x04
#define MPU9250_AK8963_YOUT_L              0x05
#define MPU9250_AK8963_YOUT_H              0x06
#define MPU9250_AK8963_ZOUT_L              0x07
#define MPU9250_AK8963_ZOUT_H              0x08
#define MPU9250_AK8963_ST2                 0x09
#define MPU9250_AK8963_CNTL                0x0A
#define MPU9250_AK8963_CNTL2               0x0B
#define MPU9250_AK8963_RSV                 0x0B //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
#define MPU9250_AK8963_ASTC                0x0C
#define MPU9250_AK8963_TS1                 0x0D //DO NOT ACCESS
#define MPU9250_AK8963_TS2                 0x0E //DO NOT ACCESS
#define MPU9250_AK8963_I2CDIS              0x0F
#define MPU9250_AK8963_ASAX                0x10
#define MPU9250_AK8963_ASAY                0x11
#define MPU9250_AK8963_ASAZ                0x12

#define MPU9250_AK8963_I2C_ADDR 0x0C
#define MPU9250_AK8963_POWER_DOWN 0x10
#define MPU9250_AK8963_FUSE_ROM_ACCESS 0x1F
#define MPU9250_AK8963_SINGLE_MEASUREMENT 0x11
#define MPU9250_AK8963_CONTINUOUS_MEASUREMENT 0x16 //MODE 2
#define MPU9250_AK8963_DATA_READY      (0x01)
#define MPU9250_AK8963_DATA_OVERRUN    (0x02)
#define MPU9250_AK8963_OVERFLOW        (0x80)
#define MPU9250_AK8963_DATA_ERROR      (0x40)
#define MPU9250_AK8963_CNTL2_SRST 0x01

//
#define MPU9250_I2C_SLV4_EN 0x80
//
#define MPU9250_I2C_IF_DIS (0x10)
#define MPU9250_I2C_MST_EN (0x20)
#define MPU9250_FIFO_RST (0x04)
#define MPU9250_FIFO_ENABLE (0x40)
//
#define MPU9250_RESET 0x80
#define MPU9250_CLOCK_MASK 0xF8
#define MPU9250_CLOCK_INTERNAL 0x00
#define MPU9250_CLOCK_PLL 0x01
#define MPU9250_CLOCK_PLLGYROZ 0x03
#define MPU9250_FS_SEL_MASK 0xE7
#define MPU9250_SLEEP_MASK 0x40
//
#define MPU9250_XYZ_GYRO 0xC7
#define MPU9250_XYZ_ACCEL 0xF8
//
#define MPU9250_RAW_RDY_EN (0x01)
#define MPU9250_RAW_DATA_RDY_INT (0x01)
#define MPU9250_FIFO_OVERFLOW (0x10)
//
#define MPU9250_INT_ANYRD_2CLEAR (0x10)
#define MPU9250_LATCH_INT_EN (0x20)
//
#define MPU9250_MAX_FIFO (1024)
#define MPU9250_FIFO_SIZE_1024  (0x40)
#define MPU9250_FIFO_SIZE_2048  (0x80)
#define MPU9250_FIFO_SIZE_4096  (0xC0)

#define MPU9250_TEMP_OUT (0x80)
#define MPU9250_GYRO_XOUT (0x40)
#define MPU9250_GYRO_YOUT (0x20)
#define MPU9250_GYRO_ZOUT (0x10)
#define MPU9250_ACCEL (0x08)

//
#define SMPLRT_DIV 0
#define MPU9250_SPIx_ADDR 0x00

#define AK8963_I2C_ADDR             ((uint8_t)0x0C)   /* 7b'0001100 */
#define AK8963_DeviceID             ((uint8_t)0x48)

/* Read-only Reg */
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)

/* Write/Read Reg */
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)

/* Read-only Reg ( ROM ) */
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)

/* Status */
#define AK8963_STATUS_DRDY          ((uint8_t)0x01)
#define AK8963_STATUS_DOR           ((uint8_t)0x02)
#define AK8963_STATUS_HOFL          ((uint8_t)0x08)

/*add*/
#define I2C_SLV0_ADDR                       0x25  
#define I2C_SLV0_REG                        0x26
#define I2C_SLV0_CTRL                       0x27 
#define I2C_SLV0_DO                         0x63 //output reg

//--------------------AK8963 reg addr------------------------//
#define MPU9250_AK8963_ADDR                 0x0C  //AKM addr
#define AK8963_WHOAMI_REG                   0x00  //AKM ID addr
#define AK8963_WHOAMI_ID                    0x48  //ID
#define AK8963_ST1_REG                      0x02  //Data Status1
#define AK8963_ST2_REG                      0x09  //Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR                      0x02
#define AK8963_ST1_DRDY                     0x01 //Data Ready
#define AK8963_ST2_BITM                     0x10
#define AK8963_ST2_HOFL                     0x08 // Magnetic sensor overflow 
#define AK8963_CNTL1_REG                    0x0A
#define AK8963_CNTL2_REG                    0x0B
#define AK8963_CNTL2_SRST                   0x01 //soft Reset

#define EXT_SENS_DATA_00    0x49  //MPU9250 IICÍâ¹ÒÆ÷¼þ¶ÁÈ¡·µ»Ø¼Ä´æÆ÷00
#define EXT_SENS_DATA_01    0x4a  //MPU9250 IICÍâ¹ÒÆ÷¼þ¶ÁÈ¡·µ»Ø¼Ä´æÆ÷01
#define EXT_SENS_DATA_02    0x4b  //MPU9250 IICÍâ¹ÒÆ÷¼þ¶ÁÈ¡·µ»Ø¼Ä´æÆ÷02
#define EXT_SENS_DATA_03    0x4c  //MPU9250 IICÍâ¹ÒÆ÷¼þ¶ÁÈ¡·µ»Ø¼Ä´æÆ÷03

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08
//////////////////////////////////////////////////////////////////////////
//
enum MPU9250_GYRO_DLPF {
    MPU9250_GYRO_DLPF_250HZ = 0,
    MPU9250_GYRO_DLPF_184HZ,
    MPU9250_GYRO_DLPF_92HZ,
    MPU9250_GYRO_DLPF_41HZ,
    MPU9250_GYRO_DLPF_20HZ,
    MPU9250_GYRO_DLPF_10HZ,
    MPU9250_GYRO_DLPF_5HZ,
    MPU9250_GYRO_DLPF_3600HZ,
    NUM_GYRO_DLPF
};

enum MPU9250_GYRO_FSR {
    MPU9250_FSR_250DPS = 0,
    MPU9250_FSR_500DPS,
    MPU9250_FSR_1000DPS,
    MPU9250_FSR_2000DPS,
    MPU9250_NUM_GYRO_FSR
};

enum MPU9250_ACCEL_DLPF {
    MPU9250_ACCEL_DLPF_460HZ = 0,
    MPU9250_ACCEL_DLPF_184HZ,
    MPU9250_ACCEL_DLPF_92HZ,
    MPU9250_ACCEL_DLPF_41HZ,
    MPU9250_ACCEL_DLPF_20HZ,
    MPU9250_ACCEL_DLPF_10HZ,
    MPU9250_ACCEL_DLPF_5HZ,
    MPU9250_ACCEL_DLPF_460HZ2,
    MPU9250_NUM_ACCEL_DLPF
};

enum MPU9250_ACCEL_FSR {
    MPU9250_FSR_2G = 0,
    MPU9250_FSR_4G,
    MPU9250_FSR_8G,
    MPU9250_FSR_16G,
    MPU9250_NUM_ACCEL_FSR
};

enum MPU9250_CLK {
    MPU9250_CLK_INTERNAL = 0,
    MPU9250_CLK_PLL,
    MPU9250_NUM_CLK
};
//////////////////////////////////////////////////////////////////////////
//low hardware functions
void MPU9250_Init(void);
u8 MPU9250_SPIx_SendByte(u8 data);
//void MPU9250_SPIx_SetDivisor(u16 data);

//
u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr);
int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data);
int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data);
int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data);

//global function use for eMPL
int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data);
int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data);
int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data);
int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data);

//data functions
//void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag);
void MPU9250_Get9AxisRawData(Axis3i16 *accel, Axis3i16 * gyro, Axis3i16 * mag);
void MPU9250_Get6AxisRawData(Axis3i16* accel, Axis3i16* gyro);
void MPU9250_Get3AxisAccelRawData(Axis3i16 * accel);
void MPU9250_Get3AxisGyroRawData(Axis3i16* gyro);
void MPU9250_Get3AxisMagnetRawData(Axis3i16 *mag);
void MPU9250_GetTemperatureRawData(long *temperature);

int drv_acc_gyro_mag_tdk_mpu9250_read(void *buf, signed long len);
//int MPU9250_IsDataReady(void);

#endif
