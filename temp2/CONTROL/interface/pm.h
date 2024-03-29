#ifndef __PM_H
#define __PM_H
#include "sys.h"
#include <stdbool.h>
#include "delay.h"
/********************************************************************************	 
 * ATKflight飞控固件
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * All rights reserved
********************************************************************************/

#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 1100
#endif
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 65535

#ifndef CURRENT_METER_SCALE
#define CURRENT_METER_SCALE 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#define BT_WRITE		0x70
#define BT_READ 		0x71
#define BT_ADDR     0x16

#define I2C_SCL_PIN         GPIO_Pin_6
#define I2C_SDA_PIN         GPIO_Pin_7
#define I2C_SCL_PORT        GPIOB
#define I2C_SDA_PORT        GPIOB

#define I2C_SCL_Set()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN, Bit_SET)
#define I2C_SCL_Clr()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN, Bit_RESET)

#define I2C_SDA_Set()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN, Bit_SET)
#define I2C_SDA_Clr()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN, Bit_RESET)

#define I2C_SDA_Get()  GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN)


enum {
  I2C_SDA_IN,
	I2C_SDA_OUT
};

enum {
	I2C_ACK,
	I2C_NACK
};

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} currentSensor_e;

typedef enum {
    BAT_CAPACITY_UNIT_MAH,
    BAT_CAPACITY_UNIT_MWH,
} batCapacityUnit_e;

typedef struct batteryConfig_s {

    struct {
        uint16_t scale;         // adjust this to match battery voltage to reported value
        uint16_t cellMax;       // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 421 (4.21V)
        uint16_t cellMin;       // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.3V)
        uint16_t cellWarning;   // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.5V)
    } voltage;

    struct {
        int16_t scale;          // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
        int16_t offset;         // offset of the current sensor in millivolt steps
        currentSensor_e type;   // type of current meter used, either ADC or virtual
    } current;

    struct {
        uint32_t value;         // mAh or mWh (see capacity.unit)
        uint32_t warning;       // mAh or mWh (see capacity.unit)
        uint32_t critical;      // mAh or mWh (see capacity.unit)
        batCapacityUnit_e unit; // Describes unit of capacity.value, capacity.warning and capacity.critical
    } capacity;

} batteryConfig_t;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} batteryState_e;

extern uint16_t vbat;
extern uint16_t vbatRaw;
extern uint16_t vbatLatestADC;
extern uint8_t batteryCellCount;
extern uint16_t batteryCriticalVoltage;
extern uint16_t batteryWarningVoltage;
extern uint16_t amperageLatestADC;
extern int32_t amperage;
extern int32_t power;
extern int32_t mAhDrawn;
extern int32_t mWhDrawn;
extern uint32_t batteryRemainingCapacity;
extern bool batteryUseCapacityThresholds;
extern bool batteryFullWhenPluggedIn;
extern batteryState_e batteryState;

uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
void batteryUpdate(uint32_t vbatTimeDelta);
void batteryInit(void);
void pmInit(void);

void currentMeterUpdate(int32_t lastUpdateAt);

void powerMeterUpdate(int32_t lastUpdateAt);

uint8_t calculateBatteryPercentage(void);

void pmTask(void *param);
float pmGetBatteryVoltage(void);

#endif /* __PM_H */
