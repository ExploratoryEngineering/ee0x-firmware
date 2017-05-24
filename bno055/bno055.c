#include "bno055.h"
#include <nrf_drv_twi.h>
#include "nrf_delay.h"


static nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);

void InitI2C()
{
    NRF_LOG_PRINTF("InitI2C\n");
 
    APP_ERROR_CHECK(nrf_drv_twi_init(&twi, NULL, NULL, NULL));
    nrf_drv_twi_enable(&twi);
}

void BNO055_SelectUnits()
{
    // Acceleration: m/s^2
    // Angular rate: DPS
    // Euler angles: Degrees
    // Temperature: Celcius

    BNO055_SelectRegisterMapPage(0);
    uint8_t cmd[2] = {UNIT_SELECTION_REGISTER, UNIT_ORI_WIN + UNIT_ACC_MSS + UNIT_GYR_DPS + UNIT_EULER_DEG + UNIT_TEMP_C};
    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 2, false));
}

// BNO055_SelectNDOFFusion selects nine degrees of freedom fusion mode.
// Data rates in NDOF mode:
// Accelerometer: 100Hz
// Magnetometer: 20Hz
// Gyro: 100Hz
void BNO055_SelectNDOFFusion()
{
    BNO055_SelectRegisterMapPage(0); 
    uint8_t cmd[2] = { OPR_MODE_REGISTER, NDOF_FUSION_MODE };
    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 2, false));
}

// Initializes I2C and IMU
void BNO055_Init()
{
    InitI2C();
    BNO055_CheckDeviceId();
    SelectInternalOscillator();
    BNO055_SelectUnits();
    BNO055_SelectNDOFFusion();
}


// Registers are organized in pages. Refer to section "4.2 Register map" in the BNO055 datasheet for more info
// https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST_BNO055_DS000_14.pdf
void BNO055_SelectRegisterMapPage(uint8_t page)
{
    static uint8_t cmd[2];
    cmd[0] = PAGE_ID_REGISTER;
    cmd[1] = page;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 2, false));
}
 
// This is just a sanity check. Verfies chip/accelerometer/magnetometer and gyro id.
// Returns true if everything checks out.
bool BNO055_CheckDeviceId()
{
    BNO055_SelectRegisterMapPage(0); 
    bool bOk = true;

    static uint8_t cmd[7];
    cmd[0] = CHIP_ID_REGISTER;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;
    cmd[6] = 0;
    
	APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 1, false));
 	APP_ERROR_CHECK(nrf_drv_twi_rx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 7));
    
    if (cmd[0] == BNO055_CHIP_ID) {
        NRF_LOG_PRINTF("BNO055 CHIP ID - OK\n"); 
    } else {
        NRF_LOG_PRINTF("BNO055 CHIP ID - ERROR. Got: %d\n", cmd[0]);
        bOk = false;
    }
    if (cmd[1] == BNO055_ACC_ID) {
        NRF_LOG_PRINTF("BNO055 ACCELEROMETER ID - OK\n");
    } else {
        NRF_LOG_PRINTF("BNO055 ACCELEROMETER ID - ERROR. Got: %d\n", cmd[1]);
        bOk = false;
    }
    if (cmd[2] == BNO055_MAG_ID) {
        NRF_LOG_PRINTF("BNO055 MAGNETOMETER ID - OK\n");
    } else {
        NRF_LOG_PRINTF("BNO055 ACCELERMAGNETOMETEROMETER ID - ERROR. Got: %d\n", cmd[2]);
        bOk = false;
    }
    if (cmd[3] == BNO055_GYRO_ID) {
      NRF_LOG_PRINTF("BNO055 GYRO ID - OK\n");
    } else {
        NRF_LOG_PRINTF("BNO055 GYRO ID - ERROR. Got: %d\n", cmd[3]);
        bOk = false;
    }
    return bOk;
}

// System calibration status mask:          0b11000000
// Gyro calibration status mask :           0b00110000
// Accelerometer calibration status mask:   0b00001100
// Magnetometer calibration status mask:    0b00000011
// 0 == Not calibrated, 3 == Fully calibrated
uint8_t BNO055_CalibrationStatus(void)
{
    BNO055_SelectRegisterMapPage(0);
    uint8_t cmd[1] = {CALIBRATION_STATUS};

    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 1, false));
    APP_ERROR_CHECK(nrf_drv_twi_rx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 1));
    return cmd[0];
}

void SelectInternalOscillator()
{
    BNO055_SelectRegisterMapPage(0);

    // CLK_SEL == 0 : Use internal oscillator
    uint8_t cmd[2] = {SYS_TRIGGER, 0};
    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 2, false));
}


void BNO055_ReadEulerAngles(double * heading, double * roll, double * pitch)
{
    uint8_t cmd[6] = {EULER_H_REGISTER_LSB, 0, 0, 0, 0, 0};
    int16_t _heading;
    int16_t _roll;
    int16_t _pitch;
    BNO055_SelectRegisterMapPage(0);

    APP_ERROR_CHECK(nrf_drv_twi_tx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 1, false));
	APP_ERROR_CHECK(nrf_drv_twi_rx(&twi, BNO055_I2C_ADDRESS, &cmd[0], 6));

    _heading = cmd[1] << 8 | cmd[0];
    _pitch = cmd[3] << 8 | cmd[2];
    _roll = cmd[5] << 8 | cmd[4];

    *heading = (double)(_heading >> 4);
    *roll = (double)(_roll  >> 4);
    *pitch = (double)(_pitch  >> 4);    
}


// Call this test function in order to verify behaviour of BNO055. The function will
// dump the Euler angle to the debug log. This function does not terminate.
void TraceEulerAngle()
{
    static char buffer[200];
    static double heading = 1;
    static double roll = 1;
    static double pitch = 1;
    while (true)
    {
        BNO055_ReadEulerAngles(&heading, &roll, &pitch);
        sprintf(buffer, "Heading: %f, Roll: %f, Pitch: %f\n", heading, roll, pitch);
        NRF_LOG_PRINTF(buffer);
        nrf_delay_ms(50);
    }
 }
