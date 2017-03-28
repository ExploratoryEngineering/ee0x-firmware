#include "co2-sensor.h"
#include <app_error.h>


#define IAQ_CORE_ADDRESS 0x5A
uint8_t IAQ_CORE_SENSOR_REGISTER = 0xB5;

#define T6713_REG_GAS_PPM  0x138B
#define T6713_ADDRESS 0x15

static nrf_drv_twi_t* iaq_twi;
static iAQSensor_t iaq;

// Configure and enable I2C communication with the sensors
void iAQCore_init(nrf_drv_twi_t* twidriver)
{
	iaq_twi = twidriver;
	APP_ERROR_CHECK(nrf_drv_twi_init(iaq_twi, NULL, NULL, NULL));
    nrf_drv_twi_enable(iaq_twi);
}

// GetSensorReading return the latest CO2 ppm sample values.
iAQSensor_t GetSensorReading()
{
	return iaq;
}

static 	char status[50];

// Read CO2 prediction from AMS iAQ-Core sensor
void Sample_iAQCoreSensor()
{
	/* 
	Important: This sensor only supports 100kHz I2C. Check TWI0_CONFIG_FREQUENCY if you are unable to communicate with the sensor.

	Data sheet: http://ams.com/eng/content/download/686523/1787697/348216
	*/

	static uint8_t data[9] = {0,0,0};
	// We're only interested in the CO2 prediction and status byte
	APP_ERROR_CHECK(nrf_drv_twi_rx(iaq_twi, IAQ_CORE_ADDRESS, &data[0], 3));
	uint16_t concentration = data[0]<<8 | data[1];
	switch (data[2])
	{
		case 0x00: sprintf(status, "OK");
				   break;
		case 0x10: sprintf(status, "WARM UP");
				   break;
		case 0x01: sprintf(status, "BUSY");
				   break;
		case 0x80: sprintf(status, "ERROR");
				   break;
		default:   sprintf(status, "0x%02x", data[2]);
				   break;
	}
	
	iaq.status = data[2];
	iaq.predIAQ = 0;
	if (iaq.status == 0)
	{
		iaq.predIAQ = concentration;
	}
	NRF_LOG_PRINTF("iAQ-Core: CO2 %d ppm (status:%s)\n", concentration, status);
}

// Read CO2 prediction from Telaire T6713 sensor
void Sample_TelaireT6713Sensor()
{
	/*
		Supports 100/400 kHz I2C
	
		Data Sheet: http://www.amphenol-sensors.com/en/component/edocman/297-telaire-t6713-series-datasheet/download?Itemid=591%20%27
	*/
	uint8_t telaire[5];
	telaire[0] = 0x04; // Read input registers
	telaire[1] = (T6713_REG_GAS_PPM >> 8);
	telaire[2] = (T6713_REG_GAS_PPM & 0xff);
	telaire[3] = 0; // input_registers_to_read_msb = 0;
	telaire[4] = 1; // cmdPacket->input_registers_to_read_lsb = 1;

	uint8_t tdata[4]  = {0,0,0, 0};
	tdata[0] = 0;
	tdata[1] = 0;
	tdata[2] = 0;
	tdata[3] = 0;

	APP_ERROR_CHECK(nrf_drv_twi_tx(iaq_twi, T6713_ADDRESS, &telaire[0], 5, false));
	APP_ERROR_CHECK(nrf_drv_twi_rx(iaq_twi, T6713_ADDRESS, &tdata[0], 4));

	uint16_t concentration = tdata[2] << 8 | tdata[3]; 
	iaq.predTelaire = concentration;

	NRF_LOG_PRINTF("Telaire CO2: %d ppm\n", concentration); 
}

void iAQCore_sampleCO2()
{
	Sample_iAQCoreSensor();
	Sample_TelaireT6713Sensor();
}

