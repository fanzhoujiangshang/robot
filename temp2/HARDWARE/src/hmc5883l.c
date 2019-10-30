  /**
  ******************************************************************************
  * @file			bsp_hmc5883l.c
  * @author		wildfireteam
  * @version	V1.0
  * @date		 	2013-07-31
  * @brief 		hmc5883l����������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨	������  STM32 F407 ������ 
  * ��̳		:http://www.firebbs.cn
  * �Ա�		:https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */  
#include "hmc5883l.h"
#include "i2c.h"
#include "delay.h"
#include "stdio.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * HMC5883L驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  �0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  �1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  �1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  �2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  �4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  �4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  �5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  �8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define MAG_ADDRESS             0x1E
#define MAG_DATA_REGISTER       0x03
#define MAG_DATA_REGISTER_SPI   (0x03 | 0x40)
#define HMC58X3_R_CONFA         0x00
#define HMC58X3_R_CONFB         0x01
#define HMC58X3_R_MODE          0x02
#define HMC58X3_R_IDA          	0x0A
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

static bool isInit = false;

static __inline int i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data) 
{
	return Sensors_I2C_WriteRegister(addr_,reg_,1,&data);

}  
static __inline int i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
	return Sensors_I2C_ReadRegister(addr_,reg_,len,buf);

}

bool hmc5883lInit(void)
{
	if (isInit) return true;
	
	I2cMaster_Init();
	
	u8 id = 0x00;
	for (int i=0; i<10; i++)
	{
//		i2cdevReadByte(I2C1_DEV, MAG_ADDRESS, HMC58X3_R_IDA, &id);//读取ID
		i2cRead(MAG_ADDRESS,HMC58X3_R_IDA,1,&id);
		
		if (id == 'H')
		{
			break;
		}
		delay_ms(10);
	}
	
	if (id == 'H')
	{
		i2cWrite(MAG_ADDRESS,HMC58X3_R_CONFA,0x78);
		delay_ms(5);
		i2cWrite(MAG_ADDRESS,HMC58X3_R_CONFB,0x20);
		delay_ms(5);
		i2cWrite(MAG_ADDRESS,HMC58X3_R_MODE,0x00);
		delay_ms(100);

		 uint8_t buf[3];

		 i2cRead(MAG_ADDRESS, HMC58X3_R_CONFA, 3, buf);	
		 printf("buf[0]=%x ,buf[1]=%x ,buf[2]=%x \n",buf[0],buf[1],buf[2]);	
	
		isInit = true;
		printf("HMC5883L I2C connection [OK].\n");
	}
    else
	{
		printf("HMC5883L I2C connection [FAIL].\n");
	}
	
	return isInit;
}

bool hmc5883lRead(Axis3i16* magRaw)
{
    uint8_t buf[6];
	
	bool ack = i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);

	magRaw->x = (int16_t)(buf[0] << 8 | buf[1]);
	magRaw->z = (int16_t)(buf[2] << 8 | buf[3]);
	magRaw->y = (int16_t)(buf[4] << 8 | buf[5]);
//	printf("x=%d ,y=%d ,z=%d \n",magRaw->x,magRaw->y,magRaw->z);	
	return ack;
}

