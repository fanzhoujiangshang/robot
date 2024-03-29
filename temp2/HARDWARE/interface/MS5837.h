#ifndef _MS5837_H_
#define _MS5837_H_

#ifdef __cplusplus
	extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>
#include "sensors_types.h"

#define MS5837_ADDR_WRITE         0xEC
#define MS5837_ADDR_READ          0xED
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

static const float Pa   = 100.0f;
static const float bar  = 0.001f;
static const float mbar = 1.0f;

typedef enum EDrvStatus
{
	DRV_SUCCESS =  0,
	DRV_FAILURE  = -1,
	
	DRV_RESET_FAILURE,
	DRV_TRANSMIT_FAILURE,
	DRV_RECIEVE_FAILURE,
	DRV_CRC_ERROR,
 
}DrvStatus;
 
typedef enum EMS5837Model
{
	MS5837_30BA = 0, // uint8_t
	MS5837_02BA = 1, // uint8_t
}MS5837Model;

typedef struct MS5837Device
{
	uint8_t i2c_port;
	MS5837Model model;	
	float fluidDensity;

	uint16_t calibData[8];
	
	uint32_t D1;
	uint32_t D2;
	int32_t  TEMP;
	int32_t  P;	
	
}MS5837Device;

void MS5837SetModel( struct MS5837Device* dev, MS5837Model model );

/* Provide the density of the working fluid in kg/m^3. Default is for 
 * seawater. Should be 997 for freshwater.
 */
void MS5837SetFluidDensity( struct MS5837Device* dev, float density );

void MS5837Init( struct MS5837Device* dev );

float MS5837Pressure( struct MS5837Device* dev, float conversion );
float MS5837Temperature( struct MS5837Device* dev );
float MS5837Depth( struct MS5837Device* dev );
float MS5837Altitude( struct MS5837Device* dev );

static void MS5837Calculate( struct MS5837Device* dev );
DrvStatus MS5837Read( struct MS5837Device* dev );
MS5837Device MS5837GetNewDevice( MS5837Model model, float density, uint8_t port);
void drv_pressure_meas_ms5837_read(void *buf);
bool drv_pressure_meas_ms5837_init(void);

//void MS5837DeleteDevice( MS5837Device* dev );

#ifdef __cplusplus
}
#endif

#endif /* _MS5837_H_ */
