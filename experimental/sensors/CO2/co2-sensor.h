
#ifndef __IAQ_CORE_H__
#define __IAQ_CORE_H__

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include <nrf_drv_twi.h>


// These are status codes returned by the iAQ-Core sensor
// Attempting to run the sensor at less than 3.3V may yield
// other (undocumented) status codes.
typedef enum {
	OK = 0x00, 		// Data is valid.
	RUNNING = 0x10, // Sensor is in warm up phase.
	BUSY = 0x01,	// Re-read multi byte data.
	ERROR = 0x80	// Error. If constant, sensor should be replaced.
} iAQCoreStatus_t;

typedef struct  {
	// CO2 prediction in ppm from iAQ-Core sensor (Digikey part # IAQ-CORE C-ND).
	uint16_t predIAQ;
	// iAQ-Core sensor status.
	iAQCoreStatus_t status;
	// CO2 prediction in ppm from Telaire T6713 sensor (Digikey part # 	235-1373-ND).
	uint16_t predTelaire;
} iAQSensor_t;

// iAQCore_init has to be called in order to configure and enable I2C .
void iAQCore_init(nrf_drv_twi_t* twidriver);

// iAQCore_sampleCO2 reads CO2 ppm levels from sensors.
void iAQCore_sampleCO2();

// GetSensorReading return the latest CO2 ppm sample values.
iAQSensor_t GetSensorReading();

// Read CO2 prediction from AMS iAQ-Core sensor
void Sample_iAQCoreSensor();

// Read CO2 prediction from Telaire T6713 sensor
void Sample_TelaireT6713Sensor();

#endif // __IAQ_CORE_H__ //
