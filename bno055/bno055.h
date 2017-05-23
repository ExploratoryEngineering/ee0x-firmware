#ifndef _BNO055_h_
#define _BNO055_h_

#include "app_error.h"

// Registers
#define CHIP_ID_REGISTER          0x00
#define PAGE_ID_REGISTER          0x07
#define OPR_MODE_REGISTER         0x3d
#define UNIT_SELECTION_REGISTER   0x3b
#define EULER_H_REGISTER_LSB      0x1a
#define EULER_H_REGISTER_MSB      0x1b
#define CALIBRATION_STATUS        0x35
#define SYS_TRIGGER               0x3F


// IDs
#define BNO055_CHIP_ID        0xa0    
#define BNO055_ACC_ID         0xfb    
#define BNO055_MAG_ID         0x32    
#define BNO055_GYRO_ID         0x0f    

// Units
#define UNIT_ACC_MSS            0x00 
#define UNIT_GYR_DPS            0x00 
#define UNIT_EULER_DEG          0x00 
#define UNIT_TEMP_C             0x00 
#define UNIT_ORI_WIN            0x00 

#define NDOF_FUSION_MODE 0x0C
#define IMU_FUSION_MODE 0x08

// I2C address is 0x28 or 0x29
#define BNO055_I2C_ADDRESS 0x28

void InitI2C();

bool BNO055_CheckDeviceId();
void BNO055_Init();
void BNO055_SelectUnits();
void BNO055_SelectNDOFFusion();
void BNO055_SelectRegisterMapPage(uint8_t page);
uint8_t BNO055_CalibrationStatus(void);
void SelectInternalOscillator();
void BNO055_ReadEulerAngles(double * heading, double * roll, double * pitch);
void TraceEulerAngle();

#endif