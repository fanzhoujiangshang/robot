#ifndef __CONFIG_H
#define __CONFIG_H
#include "nvic.h"
#include "stdio.h"	/*printf 调用*/

/********************************************************************************	 
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 
 * All rights reserved
********************************************************************************/

#define BOOTLOADER_SIZE		(64*1024)	
#define CONFIG_PARAM_SIZE	(16*1024)

#define CONFIG_PARAM_ADDR 	(FLASH_BASE + BOOTLOADER_SIZE + 0x78000)	/*16K bootloader*/
#define FIRMWARE_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 模拟eeprom*/


#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

#define P_NAME "MiniFly"
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22


#endif /* __CONFIG_H */
