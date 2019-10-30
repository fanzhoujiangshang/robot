#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "MS5837.h"
#include "i2c.h"
//#include "i2cdev.h"
//#include "myiic.h"
#include "delay.h"
#include "sensors_types.h"

uint16_t size;
uint8_t data[256];

MS5837Device ms583730ba_sensor;

static bool isInit = false;

#if 1
static  int usr_i2c_write_bytes(uint8_t addr_, uint8_t reg_) 
{
	return Sensors_I2C_WriteNoRegister(addr_,reg_);
	
//	return i2cdevWrite(I2C1_DEV, addr_, 0xff, 1, &reg_);
//	return IIC_WriteOneByte(addr_,reg_);
}  
static  int usr_i2c_read_bytes(uint8_t addr_, uint8_t len, uint8_t* buf)
{
	return Sensors_I2C_ReadNoRegister(addr_,len,buf);
//	return i2cdevRead(I2C1_DEV, addr_, 0xff, len, buf);	
//	return IIC_Read(addr_,buf,len);
}
#endif

static uint16_t to_uint16( uint8_t bytes[] )
{
  return (uint16_t)( (bytes[0] << 8) | bytes[1] );
}

static uint32_t to_uint32( uint8_t bytes[] )
{
  return (uint32_t)( (bytes[0] << 16) | (bytes[1] << 8) | bytes[2] );
}

static uint8_t crc4( uint16_t n_prom[] )
{
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0; i < 16; i++ )
	{
		if ( i%2 == 1 )
		{
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		}
		else
		{
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- )
		{
			if ( n_rem & 0x8000 )
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
 

void MS5837SetModel( struct MS5837Device* dev, MS5837Model model )
{
	dev->model = model;
}

void MS5837SetFluidDensity( struct MS5837Device* dev, float density )
{
	dev->fluidDensity = density;
}

void MS5837Init( struct MS5837Device* dev )
{
	uint8_t cmd;
	uint8_t buffer[2];
	
	DrvStatus st = DRV_FAILURE;
	
	while ( st != DRV_SUCCESS )
	{	
		// Reset the MS5837 according to datasheet
		cmd = MS5837_RESET;
		
		if ( usr_i2c_write_bytes(MS5837_ADDR_WRITE, cmd) != 0x00 )
		{
			st = DRV_RESET_FAILURE;
			size = sprintf( (char *)data, "MS5837 RESET failed\n\r" );
			printf("%s", data);
			continue;
		}
		
		// Wait for reset to complete
		delay_ms(10);

		// Read calibration values and CRC
		for ( uint8_t i = 0; i < 7; i++ )
		{
			cmd = MS5837_PROM_READ + i*2;
			if( usr_i2c_write_bytes( MS5837_ADDR_WRITE, cmd) != 0 )
			{
				st = DRV_TRANSMIT_FAILURE;
				size = sprintf( (char *)data, "MS5837 i2c TRANSMIT failed\n\r" );
				printf("%s", data);
				break;
			}
			
			if ( usr_i2c_read_bytes(MS5837_ADDR_READ, 2, buffer) != 0 )
			{
				st = DRV_RECIEVE_FAILURE;
				size = sprintf( (char *)data, "MS5837 i2c RECIEVE failed\n\r" );
				printf("%s", data);
				break;
			}
			
			dev->calibData[i] = to_uint16( buffer );
		}
		
		if ( st != DRV_FAILURE )
		{
			continue;
		}

		// Verify data with CRC
		uint8_t crcRead = dev->calibData[0] >> 12;
		uint8_t crcCalculated = crc4( dev->calibData );

		if ( crcCalculated == crcRead )
		{
			st = DRV_SUCCESS; // Initialization success
		}
		else
		{
			st = DRV_CRC_ERROR; // CRC fail
			size = sprintf( (char *)data, "MS5837 CRC error\n\r" );
			printf("%s", data);
		}
	}
	
	size = sprintf( (char *)data, "MS5837 init SUCCESS!\n\r" );
	printf("%s", data);
}

float MS5837Pressure( struct MS5837Device* dev, float conversion )
{
	return dev->P * conversion;
}

float MS5837Temperature( struct MS5837Device* dev )
{
	return dev->TEMP / 100.0f;
}

float MS5837Depth( struct MS5837Device* dev )
{
	return ( MS5837Pressure( dev, Pa ) - .101300f ) / ( dev->fluidDensity * 9.80665f );
}

float MS5837Altitude( struct MS5837Device* dev )
{
	return ( 1 - pow( ( MS5837Pressure( dev, mbar ) / 1013.25f), .190284f ) ) * 145366.45 * .3048;
}

static void MS5837Calculate( struct MS5837Device* dev )
{
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation	
	int32_t dT 		= 0;
	int64_t SENS 	= 0;
	int64_t OFF 	= 0;
	int32_t SENSi = 0;
	int32_t OFFi 	= 0;  
	int32_t Ti 		= 0;    
	int64_t OFF2 	= 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = dev->D2 - (uint32_t)(dev->calibData[5]) * 256l;
	if ( dev->model == MS5837_02BA )
	{
		SENS = (int64_t)(dev->calibData[1]) * 65536l + ( (int64_t)(dev->calibData[3]) * dT ) / 128l;
		OFF = (int64_t)(dev->calibData[2]) * 131072l + ( (int64_t)(dev->calibData[4]) * dT ) / 64l;
		dev->P = ( dev->D1 * SENS/(2097152l) - OFF ) / (32768l);
	}
	else
	{
		SENS = (int64_t)(dev->calibData[1]) * 32768l + ( (int64_t)(dev->calibData[3]) * dT ) / 256l;
		OFF = (int64_t)(dev->calibData[2]) * 65536l + ( (int64_t)(dev->calibData[4]) * dT ) / 128l;
		dev->P = ( dev->D1 * SENS / (2097152l)-OFF ) / (8192l);
	}
	
	// Temp conversion
	dev->TEMP = 2000l + (int64_t)(dT) * dev->calibData[6] / 8388608LL;
	
	//Second order compensation
	if ( dev->model == MS5837_02BA )
	{
		if ( (dev->TEMP / 100) < 20 )
		{
			//Low temp
			Ti = ( 11*(int64_t)(dT) * (int64_t)(dT) ) / (34359738368LL);
			OFFi = ( 31 * ( dev->TEMP-2000 ) * ( dev->TEMP-2000 ) ) / 8;
			SENSi = ( 63 * ( dev->TEMP-2000 ) * ( dev->TEMP-2000 ) ) / 32;
		}
	}
	else
	{
		if ( (dev->TEMP / 100) < 20 )
		{
			//Low temp
			Ti = ( 3 * (int64_t)(dT) * (int64_t)(dT) ) / (8589934592LL);
			OFFi = ( 3 * ( dev->TEMP - 2000 ) * ( dev->TEMP - 2000 ) ) / 2;
			SENSi = ( 5 * ( dev->TEMP - 2000 ) * ( dev->TEMP - 2000 ) ) / 8;
			
			if ( (dev->TEMP / 100) < -15 )
			{
				//Very low temp
				OFFi = OFFi + 7 * ( dev->TEMP + 1500l ) * ( dev->TEMP + 1500l );
				SENSi = SENSi + 4 * ( dev->TEMP + 1500l ) * ( dev->TEMP + 1500l );
			}
		}
		else if ( ( dev->TEMP / 100 ) >= 20 )
		{
			//High temp
			Ti = 2 * ( dT * dT ) / (137438953472LL);
			OFFi = ( 1 * ( dev->TEMP - 2000) * ( dev->TEMP - 2000 ) ) / 16;
			SENSi = 0;
		}
	}
	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	if ( dev->model == MS5837_02BA )
	{
		dev->TEMP = (dev->TEMP-Ti);
		dev->P = ( ( ( dev->D1 * SENS2 ) / 2097152l - OFF2 ) / 32768l ) / 100;
	}
	else
	{
		dev->TEMP = (dev->TEMP-Ti);
		dev->P = ( ( ( dev->D1 * SENS2 ) / 2097152l - OFF2 ) / 8192l ) / 10;
	}
}

DrvStatus MS5837Read( struct MS5837Device* dev )
{
	uint8_t cmd;
	uint8_t buffer[3];
	
	// Request D1 conversion
	cmd = MS5837_CONVERT_D1_8192;
	if ( usr_i2c_write_bytes(MS5837_ADDR_WRITE, cmd) != 0x00 )
	{
		return DRV_TRANSMIT_FAILURE;
	}

	delay_ms(50); // Max conversion time per datasheet
	
	cmd = MS5837_ADC_READ;
	if ( usr_i2c_write_bytes(MS5837_ADDR_WRITE, cmd) != 0 )
	{
		return DRV_TRANSMIT_FAILURE;
	}
	
	delay_ms(20); // Max conversion time per datasheet
	
	if ( usr_i2c_read_bytes(MS5837_ADDR_READ, 3, buffer) != 0 )
	{
		return DRV_RECIEVE_FAILURE;
	}
	
	dev->D1 = to_uint32( buffer );
	
	// Request D2 conversion
	cmd = MS5837_CONVERT_D2_8192;
	if ( usr_i2c_write_bytes(MS5837_ADDR_WRITE, cmd) != 0 )
	{
		return DRV_TRANSMIT_FAILURE;
	}	
	
	delay_ms(50); // Max conversion time per datasheet
	
	cmd = MS5837_ADC_READ;
	if ( usr_i2c_write_bytes(MS5837_ADDR_WRITE, cmd) != 0 )
	{
		return DRV_TRANSMIT_FAILURE;
	}
	
	delay_ms(20); // Max conversion time per datasheet	
	
	if ( usr_i2c_read_bytes(MS5837_ADDR_READ, 3, buffer) != 0 )
	{
		return DRV_RECIEVE_FAILURE;
	}

	dev->D2 = to_uint32( buffer );	

	MS5837Calculate( dev );
	
	return DRV_SUCCESS;
}

MS5837Device MS5837GetNewDevice( MS5837Model model, float density, uint8_t port)
{
	MS5837Device dev;
	dev.model = model;
	dev.fluidDensity = density;
	dev.i2c_port = port;
	
	return dev;
}

#define depth_array_space 20


void drv_pressure_meas_ms5837_read(void *buf)
{
		static float initial_depth;
		static char  depth_array_number;
		static float depth_array[depth_array_space];
		static char  depth_i;
		static float depth_array_sum;
		
    pressure_data_t *pdata = (pressure_data_t *)buf;
	
    MS5837Device* dev;

    dev = &ms583730ba_sensor;
		
    MS5837Read( dev );
		
    float pressure = MS5837Pressure( dev, Pa );
    float temperature = MS5837Temperature( dev );
    float depth = MS5837Depth( dev );
    float altitude = MS5837Altitude( dev );
		
		if(depth_array_number < depth_array_space)
		{
			depth_array[depth_array_number] = depth;
			depth_array_number++;
		}

		if(depth_array_number == depth_array_space)
		{
			depth_array_number ++;
			depth_array_sum = 0;
			for(depth_i=0;depth_i<depth_array_space;depth_i++)
			{
				depth_array_sum += depth_array[depth_i];
			}
			initial_depth = depth_array_sum/depth_array_space;
		}
		
		if(depth_array_number >= 20)
		{
			pdata->depth = (depth - initial_depth)*100;
			if(pdata->depth < 0)
			{
				pdata->depth = 0;
			}
		}
		
    pdata->pressure = pressure;
    pdata->temperature = temperature;
//    pdata->depth = depth;
    pdata->altitude = altitude;
		
//printf("----pressure=%6.2f ,temperature=%6.2f  ,depth=%6.2f -----\n",pressure, temperature,depth);

}


bool drv_pressure_meas_ms5837_init(void)
{
    if (isInit) return true;
	
    I2cMaster_Init();
//    i2cdrvInit(I2C1_DEV);
//	 IIC_Init();

    ms583730ba_sensor = MS5837GetNewDevice( MS5837_30BA, 1029, 1);	
    MS5837Init( &ms583730ba_sensor );

    printf("%s successfully \n", __func__);
    return isInit;
}
