#include "pm.h"
#include "adc1.h"
#include "stdio.h"
#include "filter.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 电源管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define ADCVREF 3300                 //单位mV (3300 = 3.3V)
#define BATTERY_SCALE	1100		 //1100 = 11:1 voltage divider (10k:1k)
#define VOLTAGE_CELLMAX 424			 //单节电池最高电压4.24V
#define VOLTAGE_CELLMIN	330			 //单节电池最高电压3.3V
#define VOLTAGE_CELLWARNING	350		 //单节低压报警值3.5V

#define VBATT_CELL_FULL_MAX_DIFF 14  // Max difference with cell max voltage for the battery to be considered full (10mV steps)
#define VBATT_PRESENT_THRESHOLD 100  // Minimum voltage to consider battery present
#define VBATT_STABLE_DELAY 40        // Delay after connecting battery to begin monitoring
#define VBATT_HYSTERESIS 10          // Batt Hysteresis of +/-100mV for changing battery state
#define VBATT_LPF_FREQ  1            // Battery voltage filtering cutoff
#define AMPERAGE_LPF_FREQ  1         // Battery current filtering cutoff


#define I2C_DELAY   200

uint8_t batteryCellCount = 3;       //默认电池节数
uint16_t batteryFullVoltage;
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;
uint32_t batteryRemainingCapacity = 0;
bool batteryUseCapacityThresholds = false;
bool batteryFullWhenPluggedIn = false;

uint16_t vbat = 0;                  // battery voltage in 0.1V steps (filtered)
uint16_t vbatLatestADC = 0;         // most recent unsmoothed raw reading from vbat ADC
uint16_t amperageLatestADC = 0;     // most recent raw reading from current ADC

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t power = 0;                  // power draw in cW (0.01W resolution)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start
int32_t mWhDrawn = 0;               // energy (milliWatt hours) drawn from the battery since start\

float battery_value;
float battery_filter1,battery_filter2;
float battery_percent;

batteryState_e batteryState;


//智能电池

void pmInitialization(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //     GPIO_Mode_Out_PP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //     GPIO_Mode_Out_PP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);
	
	I2C_SCL_Set();
	I2C_SDA_Set();
}

void I2C_SDAMode(uint8_t Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
	if (Mode)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
	else
    {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    }	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_Start(void)
{
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Set();
	I2C_SCL_Set();
	delay_us(I2C_DELAY);
	I2C_SDA_Clr();
	delay_us(I2C_DELAY);
	I2C_SCL_Clr();
	delay_us(I2C_DELAY);
}

void I2C_Stop(void)
{
	I2C_SDAMode(I2C_SDA_OUT);
	delay_us(I2C_DELAY);
	I2C_SCL_Clr();
	I2C_SDA_Clr();
	I2C_SCL_Set();
	delay_us(I2C_DELAY);
	I2C_SDA_Set();
}

bool I2C_WaitForAck(void)
{
	u8 data;
	I2C_SDAMode(I2C_SDA_IN);
	I2C_SDA_Set();
	delay_us(I2C_DELAY);
    I2C_SCL_Set();
	delay_us(I2C_DELAY);
	if(I2C_SDA_Get())
	{
		data = 1;
	}
	else 
	{
		data = 0;
	}
	I2C_SCL_Clr();
	delay_us(I2C_DELAY);
	return data;
}

void I2C_Ack(void)
{
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Clr();
	delay_us(I2C_DELAY);
	I2C_SCL_Set();
	delay_us(I2C_DELAY);
	I2C_SCL_Clr();
	delay_us(I2C_DELAY);
	I2C_SDA_Set();
}

void I2C_NAck(void)
{
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Set();
	delay_us(I2C_DELAY);
	I2C_SCL_Set();
	delay_us(I2C_DELAY);
	I2C_SCL_Clr();
}

void I2C_WriteByte(uint8_t Data)
{
	uint8_t i;
	delay_us(I2C_DELAY);
	I2C_SDAMode(I2C_SDA_OUT);
    
	for (i = 0; i < 8; i ++)
	{   
		if (Data & 0x80)
        {
			I2C_SDA_Set();
        }
		else
        {
			I2C_SDA_Clr();
        }
		delay_us(I2C_DELAY);
		I2C_SCL_Set();
		delay_us(I2C_DELAY);
		I2C_SCL_Clr();
		if(i == 7)
		{
			I2C_SDA_Set();
		}
		Data <<= 1;
		delay_us(I2C_DELAY);
	}
}

bool I2C_WriteOneByte( uint8_t RegAddr,uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(BT_WRITE);
	I2C_WaitForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaitForAck();
	I2C_WriteByte(Data);
	I2C_WaitForAck();
	I2C_Stop();	
	return true;
}

bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;	
	I2C_Start();
	I2C_WriteByte(BT_WRITE);
	I2C_WaitForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaitForAck();
	for(i = 0;i < Num;i++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaitForAck();
	}
	I2C_Stop();	
	
	return true;
}

uint8_t I2C_ReadByte(void)
{
	uint8_t i, RecDat = 0;
	I2C_SDAMode(I2C_SDA_IN);
	delay_us(I2C_DELAY);
	for(i = 0; i < 8; i ++)
	{
		RecDat <<= 1;
		I2C_SCL_Set();
		delay_us(I2C_DELAY);
		if(I2C_SDA_Get())
		{
			RecDat |= 0x01;
		}
		else
		{
			RecDat &= ~0x01;
		}
		I2C_SCL_Clr();
		delay_us(I2C_DELAY);
	}
	return RecDat;
}

bool I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t *Val)
{
	uint8_t TempVal = 0;
	I2C_Start();
	I2C_WriteByte(BT_WRITE);
	I2C_WaitForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaitForAck();
	I2C_Start();
	I2C_WriteByte(BT_READ);
	TempVal = I2C_ReadByte();
	I2C_NAck();
	I2C_Stop();	
	*Val = TempVal;
	return true;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;
	I2C_Start();
	I2C_WriteByte(BT_WRITE);
	I2C_WaitForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaitForAck();
	I2C_Start();
	I2C_WriteByte(BT_READ);
	for(i = 0;i<Num;i++)
	{
		if(i == Num - 1)
		{
			*(pBuff + i) = I2C_ReadByte();
			I2C_Ack();
		}
		else 
		{
			*(pBuff + i) = I2C_ReadByte();
			I2C_NAck();
		}
	}
	I2C_Stop();	
	return true;
}

void pmInit(void)
{
//		pmInitialization();
		adc1Init();
	
    batteryState = BATTERY_NOT_PRESENT;
    batteryCellCount = 1;
    batteryFullVoltage = 0;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
	
		
}

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // result is Vbatt in 0.01V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 1100 = 11:1 voltage divider (10k:1k)
    return((uint64_t)src * BATTERY_SCALE * ADCVREF / (0xFFF * 1000));
}

static void updateBatteryVoltage(float vbatTimeDelta)
{
    uint16_t vbatSample;
	static pt1Filter_t vbatFilterState;
	
    vbatSample = vbatLatestADC = adcValues[1];
    vbatSample = pt1FilterApply4(&vbatFilterState, vbatSample, VBATT_LPF_FREQ, vbatTimeDelta);
    vbat = batteryAdcToVoltage(vbatSample);
//	printf("adcValues[0]=%d, vbatLatestADC=%d ,vbat=%d \n",adcValues[0],vbatLatestADC,vbat);			
}

typedef struct
{
	u8 temp_data[8];
	u16 x_position;
	u16 y_position;
	u8 Reg_Addr;
	u8 status_up;
	u8 status_down;
	u8 status_press;
	u32 press_times;

}TF1touchStruct_t;
TF1touchStruct_t TF1touchdata;
/* 电源管理任务 */
void pmTask(void *param)	
{
	while(1)
	{
		vTaskDelay(100);//100ms
		updateBatteryVoltage(0.1);
		battery_value = (((float)vbat)/124.8878f);
		battery_percent = (battery_value - 18.0f)/0.072;
		if(battery_percent > 100)
		{
			battery_percent = 100;
		}
		else if(battery_percent < 0)
		{
			battery_percent = 0;
		}
		
//		I2C_Start();
//		I2C_WriteByte(0x16);
//		I2C_WaitForAck();
//		I2C_WriteByte(0x0D);//
//		I2C_WaitForAck();
//		I2C_Start();
//		I2C_WriteByte(0x17);
//		I2C_WaitForAck();
////		I2C_WriteByte(0x04);
////		I2C_WaitForAck();
//		TF1touchdata.temp_data[1] = I2C_ReadByte();
//		I2C_Ack();
//		TF1touchdata.temp_data[2] = I2C_ReadByte();
//		I2C_Ack();
//		TF1touchdata.temp_data[3] = I2C_ReadByte();
//		I2C_NAck();
//		I2C_Stop();

		/* battery has just been connected*/
		if (batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD)
		{
			/* Actual battery state is calculated below, this is really BATTERY_PRESENT */
			batteryState = BATTERY_OK;
			/* wait for VBatt to stabilise then we can calc number of cells
			(using the filtered value takes a long time to ramp up)
			We only do this on the ground so don't care if we do block, not
			worse than original code anyway*/
			updateBatteryVoltage(0.1);

			unsigned cells = (batteryAdcToVoltage(vbatLatestADC) / VOLTAGE_CELLMAX) + 1;
			if (cells > 8) cells = 8; // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)

			batteryCellCount = cells;
			batteryFullVoltage = batteryCellCount * VOLTAGE_CELLMAX;
			batteryWarningVoltage = batteryCellCount * VOLTAGE_CELLWARNING;
			batteryCriticalVoltage = batteryCellCount * VOLTAGE_CELLMIN;

			batteryFullWhenPluggedIn = batteryAdcToVoltage(vbatLatestADC) >= (batteryFullVoltage - cells * VBATT_CELL_FULL_MAX_DIFF);
		}
		/* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD */
		else if (batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD) 
		{
			batteryState = BATTERY_NOT_PRESENT;
			batteryCellCount = 0;
			batteryWarningVoltage = 0;
			batteryCriticalVoltage = 0;
		}

		if (batteryState != BATTERY_NOT_PRESENT) 
		{
			switch (batteryState)
			{
				case BATTERY_OK:
					if (vbat <= (batteryWarningVoltage - VBATT_HYSTERESIS))
						batteryState = BATTERY_WARNING;
					break;
				case BATTERY_WARNING:
					if (vbat <= (batteryCriticalVoltage - VBATT_HYSTERESIS)) 
					{
						batteryState = BATTERY_CRITICAL;
					} else if (vbat > (batteryWarningVoltage + VBATT_HYSTERESIS))
					{
						batteryState = BATTERY_OK;
					}
					break;
				case BATTERY_CRITICAL:
					if (vbat > (batteryCriticalVoltage + VBATT_HYSTERESIS))
						batteryState = BATTERY_WARNING;
					break;
				default:
					break;
			}
		}

		// handle beeper
		switch (batteryState) 
		{
			case BATTERY_WARNING:
//				beeper(BEEPER_BAT_LOW);
				break;
			case BATTERY_CRITICAL:
//				beeper(BEEPER_BAT_CRIT_LOW);
				break;
			default:
				break;
		}
	}
}

float pmGetBatteryVoltage(void)
{
	battery_value = (((float)vbat)/124.8878f);
//	return ((float)vbat/100);
 	return ((float)vbat/124.8878f);
}

batteryState_e getBatteryState(void)
{
	return batteryState;
}


