/******************************************************************************
LSM9DS1_Types.h
SFE_LSM9DS1 Library - LSM9DS1 Types and Enumerations
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 21, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file defines all types and enumerations used by the LSM9DS1 class.

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM9DS1_Types_H__
#define __LSM9DS1_Types_H__

#include <stdint.h>
#include <stdbool.h>

#include "LSM9DS1_Registers.h"

// accel_scale defines all possible FSR's of the accelerometer:
typedef enum accel_scale_e
{
	A_SCALE_2G,	// 00:  2g
	A_SCALE_16G,// 01:  16g
	A_SCALE_4G,	// 10:  4g
	A_SCALE_8G	// 11:  8g
} accel_scale_t;

// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum gyro_scale_e
{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS,	// 11:  2000 dps
} gyro_scale_t;

// mag_scale defines all possible FSR's of the magnetometer:
typedef enum mag_scale_e
{
	M_SCALE_4GS, 	// 00:  4Gs
	M_SCALE_8GS,	// 01:  8Gs
	M_SCALE_12GS,	// 10:  12Gs
	M_SCALE_16GS,	// 11:  16Gs
} mag_scale_t;

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
typedef enum gyro_odr_e
{
	//! TODO
	G_ODR_PD,	// Power down (0)
	G_ODR_149,	// 14.9 Hz (1)
	G_ODR_595,	// 59.5 Hz (2)
	G_ODR_119,	// 119 Hz (3)
	G_ODR_238,	// 238 Hz (4)
	G_ODR_476,	// 476 Hz (5)
	G_ODR_952	// 952 Hz (6)
} gyro_ord_t;

// accel_oder defines all possible output data rates of the accelerometer:
typedef enum accel_odr_e
{
	XL_POWER_DOWN, 	// Power-down mode (0x0)
	XL_ODR_10,		// 10 Hz (0x1)
	XL_ODR_50,		// 50 Hz (0x02)
	XL_ODR_119,		// 119 Hz (0x3)
	XL_ODR_238,		// 238 Hz (0x4)
	XL_ODR_476,		// 476 Hz (0x5)
	XL_ODR_952		// 952 Hz (0x6)
} accel_odr_t;

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
typedef enum accel_abw_e
{
	A_ABW_408,		// 408 Hz (0x0)
	A_ABW_211,		// 211 Hz (0x1)
	A_ABW_105,		// 105 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
} accel_abw_t;


// mag_odr defines all possible output data rates of the magnetometer:
typedef enum mag_odr_e
{
	M_ODR_0625,	// 0.625 Hz (0)
	M_ODR_125,	// 1.25 Hz (1)
	M_ODR_250,	// 2.5 Hz (2)
	M_ODR_5,	// 5 Hz (3)
	M_ODR_10,	// 10 Hz (4)
	M_ODR_20,	// 20 Hz (5)
	M_ODR_40,	// 40 Hz (6)
	M_ODR_80	// 80 Hz (7)
} mag_odr_e;

typedef enum interrupt_select_e
{
	XG_INT1 = INT1_CTRL,
	XG_INT2 = INT2_CTRL
} interrupt_select_t;

typedef enum interrupt_generators_e
{
	INT_DRDY_XL = (1<<0),	 // Accelerometer data ready (INT1 & INT2)
	INT_DRDY_G = (1<<1),	 // Gyroscope data ready (INT1 & INT2)
	INT1_BOOT = (1<<2),	 // Boot status (INT1)
	INT2_DRDY_TEMP = (1<<2),// Temp data ready (INT2)
	INT_FTH = (1<<3),		 // FIFO threshold interrupt (INT1 & INT2)
	INT_OVR = (1<<4),		 // Overrun interrupt (INT1 & INT2)
	INT_FSS5 = (1<<5),		 // FSS5 interrupt (INT1 & INT2)
	INT_IG_XL = (1<<6),	 // Accel interrupt generator (INT1)
	INT1_IG_G = (1<<7),	 // Gyro interrupt enable (INT1)
	INT2_INACT = (1<<7),	 // Inactivity interrupt output (INT2)
} interrupt_generat_t;

typedef enum accel_interrupt_generator_e
{
	XLIE_XL = (1<<0),
	XHIE_XL = (1<<1),
	YLIE_XL = (1<<2),
	YHIE_XL = (1<<3),
	ZLIE_XL = (1<<4),
	ZHIE_XL = (1<<5),
	GEN_6D = (1<<6)
} accel_interrupt_generator_t;

typedef enum gyro_interrupt_generator_e
{
	XLIE_G = (1<<0),
	XHIE_G = (1<<1),
	YLIE_G = (1<<2),
	YHIE_G = (1<<3),
	ZLIE_G = (1<<4),
	ZHIE_G = (1<<5)
} gyro_interrupt_generator_t;

typedef enum mag_interrupt_generator_e
{
	ZIEN = (1<<5),
	YIEN = (1<<6),
	XIEN = (1<<7)
} mag_interrupt_generator_t;

typedef enum h_lactive_e
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
} h_lactive_t;

typedef enum pp_od_e
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
} pp_od_t;

typedef enum fifoMode_type_e
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 5
} fifoMode_type_t;

typedef struct gyroSettings_s
{
	// Gyroscope settings:
	uint8_t enabled;
	uint16_t scale;	// Changed this to 16-bit
	uint8_t sampleRate;
	// New gyro stuff:
	uint8_t bandwidth;
	uint8_t lowPowerEnable;
	uint8_t HPFEnable;
	uint8_t HPFCutoff;
	uint8_t flipX;
	uint8_t flipY;
	uint8_t flipZ;
	uint8_t enableX;
	uint8_t enableY;
	uint8_t enableZ;
	uint8_t latchInterrupt;
} gyroSettings_t;

typedef struct deviceSettings_s
{
    uint8_t commInterface; // Can be I2C, SPI 4-wire or SPI 3-wire
    uint8_t agAddress;	// I2C address or SPI CS pin
	uint8_t mAddress;	// I2C address or SPI CS pin
} deviceSettings_t;

typedef struct accelSettings_s
{
	// Accelerometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
	// New accel stuff:
	uint8_t enableX;
	uint8_t enableY;
	uint8_t enableZ;
	int8_t  bandwidth;
	uint8_t highResEnable;
	uint8_t highResBandwidth;
} accelSettings_t;

typedef struct magSettings_s
{
	// Magnetometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
	// New mag stuff:
	uint8_t tempCompensationEnable;
	uint8_t XYPerformance;
	uint8_t ZPerformance;
	uint8_t lowPowerEnable;
	uint8_t operatingMode;
} magSettings_t;

typedef struct temperatureSettings_s
{
	// Temperature settings
    uint8_t enabled;
} temperatureSettings_t;

typedef struct IMUSettings_s
{
	deviceSettings_t device;
	gyroSettings_t gyro;
	accelSettings_t accel;
	magSettings_t mag;
	temperatureSettings_t temp;
} IMUSettings_t;

#endif
