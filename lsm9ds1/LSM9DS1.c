/******************************************************************************
SFLSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.


Re-written to plain C + use I2C under nRF52 by Ståle Dahl
<stalehd@telenordigital.com>. Uses TWI driver in blocking mode.

******************************************************************************/

#include "LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#include "app_util_platform.h"

#include "nrf52_pins.h"
#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "board.h"

#define LSM9DS1_COMMUNICATION_TIMEOUT 1000

/*
 * SPI master configuration
 */
static const nrf_drv_spi_t spi_master_2 = NRF_DRV_SPI_INSTANCE(1);
const float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

uint16_t LSM9DS1_init(imu_t* imu, const imu_config_t* config)
{
	imu->settings.device.agAddress = IMU_CS_AG_PIN;
	imu->settings.device.mAddress = IMU_CS_M_PIN;

	imu->settings.gyro.enabled = config->enable_gyro;
	imu->settings.gyro.enableX = config->enable_gyro;
	imu->settings.gyro.enableY = config->enable_gyro;
	imu->settings.gyro.enableZ = config->enable_gyro;
	// gyro scale can be 245, 500, or 2000
	imu->settings.gyro.scale = 245;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	imu->settings.gyro.sampleRate = 6;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	imu->settings.gyro.bandwidth = 0;
	imu->settings.gyro.lowPowerEnable = config->low_power_mode;
	imu->settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	imu->settings.gyro.HPFCutoff = 0;
	imu->settings.gyro.flipX = false;
	imu->settings.gyro.flipY = false;
	imu->settings.gyro.flipZ = false;
	imu->settings.gyro.latchInterrupt = true;

	imu->settings.accel.enabled = config->enable_accel;
	imu->settings.accel.enableX = config->enable_accel;
	imu->settings.accel.enableY = config->enable_accel;
	imu->settings.accel.enableZ = config->enable_accel;
	// accel scale can be 2, 4, 8, or 16
	imu->settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	imu->settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3.
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	imu->settings.accel.bandwidth = -1;
	imu->settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	imu->settings.accel.highResBandwidth = 0;

	imu->settings.mag.enabled = config->enable_mag;
	// mag scale can be 4, 8, 12, or 16
	imu->settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	imu->settings.mag.sampleRate = 7;
	imu->settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	imu->settings.mag.XYPerformance = config->low_power_mode ? 0 : 3;
	imu->settings.mag.ZPerformance = config->low_power_mode ? 0 : 3;
	imu->settings.mag.lowPowerEnable = config->low_power_mode;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	imu->settings.mag.operatingMode = 0;

	imu->settings.temp.enabled = true;
	for (int i=0; i<3; i++)
	{
		imu->gBias[i] = 0;
		imu->aBias[i] = 0;
		imu->mBias[i] = 0;
		imu->gBiasRaw[i] = 0;
		imu->aBiasRaw[i] = 0;
		imu->mBiasRaw[i] = 0;
	}
	imu->_autoCalc = false;

	LSM9DS1_constrainScales(imu);
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	LSM9DS1_calcgRes(imu); // Calculate DPS / ADC tick, stored in gRes variable
	LSM9DS1_calcmRes(imu); // Calculate Gs / ADC tick, stored in mRes variable
	LSM9DS1_calcaRes(imu); // Calculate g / ADC tick, stored in aRes variable

	// Now, initialize our hardware interface.
	LSM9DS1_initSPI();

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	NRF_LOG("Read whoami for M\n");
	uint8_t mTest = LSM9DS1_mReadByte(imu, WHO_AM_I_M);		// Read the gyro WHO_AM_I
	NRF_LOG_PRINTF("Got %02x\n", mTest);
	NRF_LOG("Read whoami for XG\n");
	uint8_t xgTest = LSM9DS1_xgReadByte(imu, WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	NRF_LOG_PRINTF("Got %02x\n", xgTest);
	uint16_t whoAmICombined = (xgTest << 8) | mTest;

	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP)) {
		NRF_LOG("Couldn't make sense of output!\n");
		return 0;
	}

	// Gyro initialization stuff:
	LSM9DS1_initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.

	// Accelerometer initialization stuff:
	LSM9DS1_initAccel(imu); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Magnetometer initialization stuff:
	LSM9DS1_initMag(imu); // "Turn on" all axes of the mag. Set up interrupts, etc.

	if (config->calibrate) {
		if (config->enable_accel || config->enable_gyro) {
		    LSM9DS1_calibrate(imu, true);
		}
		if (config->enable_mag) {
		    LSM9DS1_calibrateMag(imu, true);
		}
	}
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void LSM9DS1_initGyro(imu_t* imu)
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection

	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (imu->settings.gyro.enabled)
	{
		tempRegValue = (imu->settings.gyro.sampleRate & 0x07) << 5;
	}
	switch (imu->settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 3);
			break;
		case 2000:
			tempRegValue |= (0x3 << 3);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (imu->settings.gyro.bandwidth & 0x3);
	LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, tempRegValue);

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	LSM9DS1_xgWriteByte(imu, CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = imu->settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (imu->settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (imu->settings.gyro.HPFCutoff & 0x0F);
	}
	LSM9DS1_xgWriteByte(imu, CTRL_REG3_G, tempRegValue);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (imu->settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (imu->settings.gyro.enableY) tempRegValue |= (1<<4);
	if (imu->settings.gyro.enableX) tempRegValue |= (1<<3);
	if (imu->settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	LSM9DS1_xgWriteByte(imu, CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (imu->settings.gyro.flipX) tempRegValue |= (1<<5);
	if (imu->settings.gyro.flipY) tempRegValue |= (1<<4);
	if (imu->settings.gyro.flipZ) tempRegValue |= (1<<3);
	LSM9DS1_xgWriteByte(imu, ORIENT_CFG_G, tempRegValue);
}

void LSM9DS1_initAccel(imu_t* imu)
{
	uint8_t tempRegValue = 0;

	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (imu->settings.accel.enableZ) tempRegValue |= (1<<5);
	if (imu->settings.accel.enableY) tempRegValue |= (1<<4);
	if (imu->settings.accel.enableX) tempRegValue |= (1<<3);

	LSM9DS1_xgWriteByte(imu, CTRL_REG5_XL, tempRegValue);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (imu->settings.accel.enabled)
	{
		tempRegValue |= (imu->settings.accel.sampleRate & 0x07) << 5;
	}
	switch (imu->settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (imu->settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (imu->settings.accel.bandwidth & 0x03);
	}
	LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, tempRegValue);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (imu->settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (imu->settings.accel.highResBandwidth & 0x3) << 5;
	}
	LSM9DS1_xgWriteByte(imu, CTRL_REG7_XL, tempRegValue);
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void LSM9DS1_calibrate(imu_t* imu, bool autoCalc)
{
//	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};

	// Turn on FIFO and set threshold to 32 samples
	LSM9DS1_enableFIFO(imu, true);
	LSM9DS1_setFIFO(imu, FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (LSM9DS1_xgReadByte(imu, FIFO_SRC) & 0x3F); // Read number of stored samples
	}
	for(ii = 0; ii < samples ; ii++)
	{	// Read the gyro data stored in the FIFO
		LSM9DS1_readGyro(imu);
		gBiasRawTemp[0] += imu->gx;
		gBiasRawTemp[1] += imu->gy;
		gBiasRawTemp[2] += imu->gz;
		LSM9DS1_readAccel(imu);
		aBiasRawTemp[0] += imu->ax;
		aBiasRawTemp[1] += imu->ay;
		aBiasRawTemp[2] += imu->az - (int16_t)(1./imu->aRes); // Assumes sensor facing up!
	}
	for (ii = 0; ii < 3; ii++)
	{
		imu->gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		imu->gBias[ii] = LSM9DS1_calcGyro(imu, imu->gBiasRaw[ii]);
		imu->aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		imu->aBias[ii] = LSM9DS1_calcAccel(imu, imu->aBiasRaw[ii]);
	}

	LSM9DS1_enableFIFO(imu, false);
	LSM9DS1_setFIFO(imu, FIFO_OFF, 0x00);

	if (autoCalc) imu->_autoCalc = true;
}

void LSM9DS1_calibrateMag(imu_t* imu, bool loadIn)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior

	for (i=0; i<128; i++)
	{
		while (!LSM9DS1_magAvailable(imu))
			;
		LSM9DS1_readMag(imu);
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = imu->mx;
		magTemp[1] = imu->my;
		magTemp[2] = imu->mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		imu->mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		imu->mBias[j] = LSM9DS1_calcMag(imu, imu->mBiasRaw[j]);
		if (loadIn)
			LSM9DS1_magOffset(imu, j, imu->mBiasRaw[j]);
	}

}
void LSM9DS1_magOffset(imu_t* imu, uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	LSM9DS1_mWriteByte(imu, OFFSET_X_REG_L_M + (2 * axis), lsb);
	LSM9DS1_mWriteByte(imu, OFFSET_X_REG_H_M + (2 * axis), msb);
}

void LSM9DS1_initMag(imu_t* imu)
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (imu->settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (imu->settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (imu->settings.mag.sampleRate & 0x7) << 2;
	LSM9DS1_mWriteByte(imu, CTRL_REG1_M, tempRegValue);

	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (imu->settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	LSM9DS1_mWriteByte(imu, CTRL_REG2_M, tempRegValue); // +/-4Gauss

	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (imu->settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (imu->settings.mag.operatingMode & 0x3);
	LSM9DS1_mWriteByte(imu, CTRL_REG3_M, tempRegValue); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (imu->settings.mag.ZPerformance & 0x3) << 2;
	LSM9DS1_mWriteByte(imu, CTRL_REG4_M, tempRegValue);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	LSM9DS1_mWriteByte(imu, CTRL_REG5_M, tempRegValue);
}

uint8_t LSM9DS1_accelAvailable(imu_t* imu)
{
	uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

	return (status & (1<<0));
}

uint8_t LSM9DS1_gyroAvailable(imu_t* imu)
{
	uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

	return ((status & (1<<1)) >> 1);
}

uint8_t LSM9DS1_tempAvailable(imu_t* imu)
{
	uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

	return ((status & (1<<2)) >> 2);
}

uint8_t LSM9DS1_magAvailable(imu_t* imu)
{
	lsm9ds1_axis_t axis = ALL_AXIS;
	uint8_t status;
	status = LSM9DS1_mReadByte(imu, STATUS_REG_M);

	return ((status & (1<<axis)) >> axis);
}

void LSM9DS1_readAccel(imu_t* imu)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	LSM9DS1_xgReadBytes(imu, OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
	imu->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	imu->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	imu->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
	if (imu->_autoCalc)
	{
		imu->ax -= imu->aBiasRaw[X_AXIS];
		imu->ay -= imu->aBiasRaw[Y_AXIS];
		imu->az -= imu->aBiasRaw[Z_AXIS];
	}
}

void LSM9DS1_readMag(imu_t* imu)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	LSM9DS1_mReadBytes(imu, OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	imu->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	imu->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	imu->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS1_readTemp(imu_t* imu)
{
	uint8_t low = LSM9DS1_xgReadByte(imu, OUT_TEMP_L);
	uint8_t high = LSM9DS1_xgReadByte(imu, OUT_TEMP_H);

	imu->temperature = (int16_t)(high << 8) | low;
}

void LSM9DS1_readGyro(imu_t* imu)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	LSM9DS1_xgReadBytes(imu, OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	imu->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	imu->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	imu->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
	if (imu->_autoCalc)
	{
		imu->gx -= imu->gBiasRaw[X_AXIS];
		imu->gy -= imu->gBiasRaw[Y_AXIS];
		imu->gz -= imu->gBiasRaw[Z_AXIS];
	}
}

#define TEMP_SCALE 16.0
#define TEMP_BIAS 27.5

float LSM9DS1_calcTemp(const imu_t* imu) {
	return (imu->temperature / TEMP_SCALE) + TEMP_BIAS;
}

float LSM9DS1_calcGyro(const imu_t* imu, int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return imu->gRes * gyro;
}

float LSM9DS1_calcAccel(const imu_t* imu, int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return imu->aRes * accel;
}

float LSM9DS1_calcMag(const imu_t* imu, int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return imu->mRes * mag;
}

void LSM9DS1_setGyroScale(imu_t* imu, uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = LSM9DS1_xgReadByte(imu, CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			imu->settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			imu->settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			imu->settings.gyro.scale = 245;
			break;
	}
	LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, ctrl1RegValue);

	LSM9DS1_calcgRes(imu);
}

void LSM9DS1_setAccelScale(imu_t* imu, uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = LSM9DS1_xgReadByte(imu, CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;

	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			imu->settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			imu->settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			imu->settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			imu->settings.accel.scale = 2;
			break;
	}
	LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, tempRegValue);

	// Then calculate a new aRes, which relies on aScale being set correctly:
	LSM9DS1_calcaRes(imu);
}

void LSM9DS1_setMagScale(imu_t* imu, uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = LSM9DS1_mReadByte(imu, CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);

	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		imu->settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		imu->settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		imu->settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		imu->settings.mag.scale = 4;
		break;
	}

	// And write the new register value back into CTRL_REG6_XM:
	LSM9DS1_mWriteByte(imu, CTRL_REG2_M, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	LSM9DS1_calcmRes(imu);
}

void LSM9DS1_setGyroODR(imu_t* imu, uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		imu->settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, temp);
	}
}

void LSM9DS1_setAccelODR(imu_t* imu, uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		imu->settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, temp);
	}
}

void LSM9DS1_setMagODR(imu_t* imu, uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = LSM9DS1_mReadByte(imu, CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	imu->settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	LSM9DS1_mWriteByte(imu, CTRL_REG1_M, temp);
}

void LSM9DS1_calcgRes(imu_t* imu)
{
	imu->gRes = ((float) imu->settings.gyro.scale) / 32768.0;
}

void LSM9DS1_calcaRes(imu_t* imu)
{
	imu->aRes = ((float) imu->settings.accel.scale) / 32768.0;
}

void LSM9DS1_calcmRes(imu_t* imu)
{
	//mRes = ((float) settings.mag.scale) / 32768.0;
	switch (imu->settings.mag.scale)
	{
	case 4:
		imu->mRes = magSensitivity[0];
		break;
	case 8:
		imu->mRes = magSensitivity[1];
		break;
	case 12:
		imu->mRes = magSensitivity[2];
		break;
	case 16:
		imu->mRes = magSensitivity[3];
		break;
	}

}

void LSM9DS1_configInt(imu_t* imu, interrupt_select_t interrupt, uint8_t generator,
	                     h_lactive_t activeLow, pp_od_t pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	LSM9DS1_xgWriteByte(imu, interrupt, generator);

	// Configure CTRL_REG8
	uint8_t temp;
	temp = LSM9DS1_xgReadByte(imu, CTRL_REG8);

	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);

	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);

	LSM9DS1_xgWriteByte(imu, CTRL_REG8, temp);
}

void LSM9DS1_configInactivity(imu_t* imu, uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;

	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	LSM9DS1_xgWriteByte(imu, ACT_THS, temp);

	LSM9DS1_xgWriteByte(imu, ACT_DUR, duration);
}

uint8_t LSM9DS1_getInactivity(imu_t* imu)
{
	uint8_t temp = LSM9DS1_xgReadByte(imu, STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void LSM9DS1_configAccelInt(imu_t* imu, uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	LSM9DS1_xgWriteByte(imu, INT_GEN_CFG_XL, temp);
}

void LSM9DS1_configAccelThs(imu_t* imu, uint8_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	LSM9DS1_xgWriteByte(imu, INT_GEN_THS_X_XL + axis, threshold);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	LSM9DS1_xgWriteByte(imu, INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1_getAccelIntSrc(imu_t* imu)
{
	uint8_t intSrc = LSM9DS1_xgReadByte(imu, INT_GEN_SRC_XL);

	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}

void LSM9DS1_configGyroInt(imu_t* imu, uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	LSM9DS1_xgWriteByte(imu, INT_GEN_CFG_G, temp);
}

void LSM9DS1_configGyroThs(imu_t* imu, int16_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	LSM9DS1_xgWriteByte(imu, INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	LSM9DS1_xgWriteByte(imu, INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	LSM9DS1_xgWriteByte(imu, INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1_getGyroIntSrc(imu_t* imu)
{
	uint8_t intSrc = LSM9DS1_xgReadByte(imu, INT_GEN_SRC_G);

	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}

void LSM9DS1_configMagInt(imu_t* imu, uint8_t generator, h_lactive_t activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);

	LSM9DS1_mWriteByte(imu, INT_CFG_M, config);
}

void LSM9DS1_configMagThs(imu_t* imu, uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	LSM9DS1_mWriteByte(imu, INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	LSM9DS1_mWriteByte(imu, INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t LSM9DS1_getMagIntSrc(imu_t* imu)
{
	uint8_t intSrc = LSM9DS1_mReadByte(imu, INT_SRC_M);

	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}

	return 0;
}

void LSM9DS1_sleepGyro(imu_t* imu, bool enable)
{
	uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	LSM9DS1_xgWriteByte(imu, CTRL_REG9, temp);
}

void LSM9DS1_enableFIFO(imu_t* imu, bool enable)
{
	uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	LSM9DS1_xgWriteByte(imu, CTRL_REG9, temp);
}

void LSM9DS1_setFIFO(imu_t* imu, fifoMode_type_t fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	LSM9DS1_xgWriteByte(imu, FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1_getFIFOSamples(imu_t* imu)
{
	return (LSM9DS1_xgReadByte(imu, FIFO_SRC) & 0x3F);
}

void LSM9DS1_constrainScales(imu_t* imu)
{
	if ((imu->settings.gyro.scale != 245) && (imu->settings.gyro.scale != 500) &&
		(imu->settings.gyro.scale != 2000))
	{
		imu->settings.gyro.scale = 245;
	}

	if ((imu->settings.accel.scale != 2) && (imu->settings.accel.scale != 4) &&
		(imu->settings.accel.scale != 8) && (imu->settings.accel.scale != 16))
	{
		imu->settings.accel.scale = 2;
	}

	if ((imu->settings.mag.scale != 4) && (imu->settings.mag.scale != 8) &&
		(imu->settings.mag.scale != 12) && (imu->settings.mag.scale != 16))
	{
		imu->settings.mag.scale = 4;
	}
}

void LSM9DS1_xgWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data)
{
	LSM9DS1_SPIwriteByte(imu->settings.device.agAddress, subAddress, data);
}

void LSM9DS1_mWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data)
{
	return LSM9DS1_SPIwriteByte(imu->settings.device.mAddress, subAddress, data);
}

uint8_t LSM9DS1_xgReadByte(imu_t* imu, uint8_t subAddress)
{
	return LSM9DS1_SPIreadByte(imu, imu->settings.device.agAddress, subAddress);
}

void LSM9DS1_xgReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	LSM9DS1_SPIreadBytes(imu, imu->settings.device.agAddress, subAddress, dest, count);
}

uint8_t LSM9DS1_mReadByte(imu_t* imu, uint8_t subAddress)
{
	return LSM9DS1_SPIreadByte(imu, imu->settings.device.mAddress, subAddress);
}

void LSM9DS1_mReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	LSM9DS1_SPIreadBytes(imu, imu->settings.device.mAddress, subAddress, dest, count);
}

void LSM9DS1_initSPI() {

	NRF_LOG("Init SPI\n");

	nrf_drv_gpiote_out_config_t outconfig = GPIOTE_CONFIG_OUT_SIMPLE(true);

    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(IMU_CS_AG_PIN, &outconfig));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(IMU_CS_M_PIN, &outconfig));
    nrf_drv_gpiote_out_set(IMU_CS_AG_PIN);
    nrf_drv_gpiote_out_set(IMU_CS_M_PIN);

	uint32_t err_code;
    nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG(1);
/*    {
        //.irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_4M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };*/
    config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    config.sck_pin = PLAT_SPI_SCK;
    config.mosi_pin = PLAT_SPI_MOSI;
    config.miso_pin = PLAT_SPI_MISO;
    err_code = nrf_drv_spi_init(&spi_master_2, &config, NULL);
    APP_ERROR_CHECK(err_code);

	NRF_LOG("Init SPI completed\n");
}

void LSM9DS1_SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data)
{
//	digitalWrite(csPin, LOW); // Initiate communication
	GpioWrite(&csPin, 0);

	uint8_t retval = 0;
	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	uint8_t outval = subAddress & 0x3F;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_2, &outval, sizeof(outval), &retval, sizeof(retval)));
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_2, &data, sizeof(data), &retval, sizeof(retval)));

//	digitalWrite(csPin, HIGH); // Close communication
	GpioWrite(&csPin, 1);
}

void LSM9DS1_SPIreadBytes(imu_t* imu, uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{
	// To indicate a read, set bit 0 (msb) of first byte to 1
	uint8_t rAddress = 0x80 | (subAddress & 0x3F);
	// Mag SPI port is different. If we're reading multiple bytes,
	// set bit 1 to 1. The remaining six bytes are the address to be read
	if ((csPin == imu->settings.device.mAddress) && count > 1) {
		rAddress |= 0x40;
	}

	uint8_t retval = 0;
//	digitalWrite(csPin, LOW); // Initiate communication
	GpioWrite(&csPin, 0);
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_2, &rAddress, sizeof(rAddress), &retval, sizeof(retval)));
	for (int i=0; i<count; i++)
	{
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_2, &rAddress, sizeof(rAddress), &retval, sizeof(retval)));
		dest[i] = retval;
	}
//	digitalWrite(csPin, HIGH); // Close communication
	GpioWrite(&csPin, 1);
}

uint8_t LSM9DS1_SPIreadByte(imu_t* imu, uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte.
	// Value is returned to `temp`.
	LSM9DS1_SPIreadBytes(imu, csPin, subAddress, &temp, 1);
	return temp;
}
