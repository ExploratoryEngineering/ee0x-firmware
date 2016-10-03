/******************************************************************************
SFE_LSM9DS1.h
SFE_LSM9DS1 Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
Magnetometer registers).

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __SparkFunLSM9DS1_H__
#define __SparkFunLSM9DS1_H__

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

//#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
//#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

typedef enum lsm9ds1_axis_e {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
} lsm9ds1_axis_t;

typedef struct imu_s {
	IMUSettings_t settings;
	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
	int16_t temperature; // Chip temperature
	float gBias[3], aBias[3], mBias[3];
	int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

	// Internal members

	// gRes, aRes, and mRes store the current resolution for each sensor.
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;

	// _autoCalc keeps track of whether we're automatically subtracting off
	// accelerometer and gyroscope bias calculated in calibrate().
	bool _autoCalc;

} imu_t;

/**! * Configuration struct
 */
typedef struct imu_config_s {
	bool enable_accel; // Enable accelerometer
	bool enable_gyro; // Enable gyro
	bool enable_mag; // Enable magnetometer
	uint8_t ag_addr; // I2C address for accelerometer and gyro
	uint8_t mag_addr; // I2C address for magnetometer
	bool calibrate; // Calibrate when starting up
	bool low_power_mode; // enable low power mode
} imu_config_t;

//bool LSM9DS1_init(imu_t* imu, const imu_config_t* config);

// LSM9DS1 -- LSM9DS1 class constructor
// The constructor will set up a handful of private variables, and set the
// communication mode as well.
uint16_t LSM9DS1_init(imu_t* imu, const imu_config_t* config);


void LSM9DS1_calibrate(imu_t* imu, bool autoCalc);
void LSM9DS1_calibrateMag(imu_t* imu, bool loadIn);
void LSM9DS1_magOffset(imu_t* imu, uint8_t axis, int16_t offset);

// accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t LSM9DS1_accelAvailable(imu_t* imu);

// gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t LSM9DS1_gyroAvailable(imu_t* imu);

// gyroAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t LSM9DS1_tempAvailable(imu_t* imu);

// magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//	  on one specific axis. Or ALL_AXIS (default) to check for new data
//	  on all axes.
// Output:	1 - New data available
//			0 - No new data available
uint8_t LSM9DS1_magAvailable(imu_t* imu);

// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void LSM9DS1_readGyro(imu_t* imu);

// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void LSM9DS1_readAccel(imu_t* imu);

// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void LSM9DS1_readMag(imu_t* imu);

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
void LSM9DS1_readTemp(imu_t* imu);

// calcTemp() - calculate temperature from raw value
float LSM9DS1_calcTemp(const imu_t* imu);

// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float LSM9DS1_calcGyro(const imu_t* imu, int16_t gyro);

// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float LSM9DS1_calcAccel(const imu_t* imu, int16_t accel);

// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float LSM9DS1_calcMag(const imu_t* imu, int16_t mag);

// setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to
// 245, 500, or 200 degrees per second.
// Input:
// 	- gScl = The desired gyroscope scale. Must be one of three possible
//		values from the gyro_scale.
void LSM9DS1_setGyroScale(imu_t* imu, uint16_t gScl);

// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale.
void LSM9DS1_setAccelScale(imu_t* imu, uint8_t aScl);

// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//		values from the mag_scale.
void LSM9DS1_setMagScale(imu_t* imu, uint8_t mScl);

// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
void LSM9DS1_setGyroODR(imu_t* imu, uint8_t gRate);

// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
void LSM9DS1_setAccelODR(imu_t* imu, uint8_t aRate);

// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
void LSM9DS1_setMagODR(imu_t* imu, uint8_t mRate);

// configInactivity() -- Configure inactivity interrupt parameters
// Input:
//	- duration = Inactivity duration - actual value depends on gyro ODR
//	- threshold = Activity Threshold
//	- sleepOn = Gyroscope operating mode during inactivity.
//	  true: gyroscope in sleep mode
//	  false: gyroscope in power-down
void LSM9DS1_configInactivity(imu_t* imu, uint8_t duration, uint8_t threshold, bool sleepOn);

// configAccelInt() -- Configure Accelerometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
//	- andInterrupts = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
void LSM9DS1_configAccelInt(imu_t* imu, uint8_t generator, bool andInterrupts);

// configAccelThs() -- Configure the threshold of an accelereomter axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-255.
//	  Multiply by 128 to get the actual raw accel value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void LSM9DS1_configAccelThs(imu_t* imu, uint8_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait);

// configGyroInt() -- Configure Gyroscope Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
//	- aoi = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
//	- latch: latch gyroscope interrupt request.
void LSM9DS1_configGyroInt(imu_t* imu, uint8_t generator, bool aoi, bool latch);

// configGyroThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw gyroscope value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void LSM9DS1_configGyroThs(imu_t* imu, int16_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait);

// configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
// Input:
//	- interrupt = Select INT1 or INT2
//	  Possible values: XG_INT1 or XG_INT2
//	- generator = Or'd combination of interrupt generators.
//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- pushPull =  Push-pull or open drain interrupt configuration
//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
void LSM9DS1_configInt(imu_t* imu, interrupt_select_t interupt, uint8_t generator,
			   h_lactive_t activeLow/* = INT_ACTIVE_LOW */, pp_od_t pushPull /*= INT_PUSH_PULL*/);

// configMagInt() -- Configure Magnetometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZIEN, YIEN, XIEN
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- latch: latch gyroscope interrupt request.
void LSM9DS1_configMagInt(imu_t* imu, uint8_t generator, h_lactive_t activeLow, bool latch);

// configMagThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw magnetometer value.
void LSM9DS1_configMagThs(imu_t* imu, uint16_t threshold);

// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
uint8_t LSM9DS1_getGyroIntSrc(imu_t* imu);

// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
uint8_t LSM9DS1_getAccelIntSrc(imu_t* imu);

// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
uint8_t LSM9DS1_getMagIntSrc(imu_t* imu);

// getGyroIntSrc() -- Get status of inactivity interrupt
uint8_t LSM9DS1_getInactivity(imu_t* imu);

// sleepGyro() -- Sleep or wake the gyroscope
// Input:
//	- enable: True = sleep gyro. False = wake gyro.
void LSM9DS1_sleepGyro(imu_t* imu, bool enable);

// enableFIFO() - Enable or disable the FIFO
// Input:
//	- enable: true = enable, false = disable.
void LSM9DS1_enableFIFO(imu_t* imu, bool enable);

// setFIFO() - Configure FIFO mode and Threshold
// Input:
//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//	- fifoThs: FIFO threshold level setting
//	  Any value from 0-0x1F is acceptable.
void LSM9DS1_setFIFO(imu_t* imu, fifoMode_type_t fifoMode, uint8_t fifoThs);

// getFIFOSamples() - Get number of FIFO samples
uint8_t LSM9DS1_getFIFOSamples(imu_t* imu);



/* ========================================================================== */

// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void LSM9DS1_initGyro();

// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void LSM9DS1_initAccel(imu_t* imu);

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void LSM9DS1_initMag(imu_t* imu);

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
uint8_t LSM9DS1_mReadByte(imu_t* imu, uint8_t subAddress);

// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void LSM9DS1_mReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void LSM9DS1_mWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
uint8_t LSM9DS1_xgReadByte(imu_t* imu, uint8_t subAddress);

// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void LSM9DS1_xgReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void LSM9DS1_xgWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data);

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void LSM9DS1_calcgRes(imu_t* imu);

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void LSM9DS1_calcmRes(imu_t* imu);

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void LSM9DS1_calcaRes(imu_t* imu);

//////////////////////
// Helper Functions //
//////////////////////
void LSM9DS1_constrainScales(imu_t* imu);


///////////////////
// SPI Functions //
///////////////////
// initSPI() -- Initialize the SPI hardware.
// This function will setup all SPI pins and related hardware.
void LSM9DS1_initSPI();

// SPIwriteByte() -- Write a byte out of SPI to a register in the device
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void LSM9DS1_SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data);

// SPIreadByte() -- Read a single byte from a register over SPI.
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t LSM9DS1_SPIreadByte(imu_t* imu, uint8_t csPin, uint8_t subAddress);

// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- csPin = The chip select pin of a slave device.
//	- subAddress = The register to begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
void LSM9DS1_SPIreadBytes(imu_t* imu, uint8_t csPin, uint8_t subAddress,
						uint8_t * dest, uint8_t count);
#endif // SFE_LSM9DS1_H //
