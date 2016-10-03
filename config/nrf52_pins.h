/*
**   Copyright 2016 Telenor Digital AS
**
**  Licensed under the Apache License, Version 2.0 (the "License");
**  you may not use this file except in compliance with the License.
**  You may obtain a copy of the License at
**
**      http://www.apache.org/licenses/LICENSE-2.0
**
**  Unless required by applicable law or agreed to in writing, software
**  distributed under the License is distributed on an "AS IS" BASIS,
**  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**  See the License for the specific language governing permissions and
**  limitations under the License.
*/

#ifndef NRF52_PINS_H
#define NRF52_PINS_H

//#define NRF52_DK 1
#define EE02

// TPS22994 is set up the following way
// Channel 1: GPS main power
// Channel 2: GPS VBackup (always on)
// Channel 3: Accelerometer/gyro
// Channel 4: Accelerometer/gyro IO (turn on )

#define TPS22994_GPS_PIN 1
#define TPS22994_VBACKUP_PIN 2
#define TPS22994_MEMS_PIN 3
#define TPS22994_MEMS_IO_PIN 4

#ifdef EE02
    //Platform wide pins
    //I2C
    #define PLAT_I2C_SDA 26
    #define PLAT_I2C_SCL 27

    //SPI (chipselect pr component)
    #define PLAT_SPI_MISO 24
    #define PLAT_SPI_MOSI 23
    #define PLAT_SPI_SCK 25

    // sx1276 pins
    #define SX1276_RXTX_PIN 2
    #define SX1276_RST_PIN 3
    #define SX1276_NSS_PIN 22
    #define SX1276_DIO0_PIN 13
    #define SX1276_DIO1_PIN 14
    #define SX1276_DIO2_PIN 15
    #define SX1276_DIO3_PIN 16
    #define SX1276_DIO4_A_PIN 19
    #define SX1276_DIO4_B_PIN 20

    #define SX1276_ANT_HF_CTRL 31
    #define SX1276_ANT_LF_CTRL 30

    #define GPS_RX_PIN 29
    #define GPS_TX_PIN 28
    #define GPS_CTS_PIN -1
    #define GPS_RTS_PIN -1

    // SPI pins for IMU

    #define IMU_CS_M_PIN 6
    #define IMU_CS_AG_PIN 5

#endif
// Configuration for nRF52 DK with sheild and breakout boards.

// The following pinout works if you are using the nRF52 DK with a Semtech Sx1276 shield 
// plus a GPS (G-Top Firely X1 or similar) connected to UART and a LSM9DS1 breakout from SparkFun.
#ifdef NRF52_DK
    // sx1276 pins
    #define SX1276_RXTX_PIN 30
    #define SX1276_RST_PIN 3
    #define SX1276_SCK_PIN 25
    #define SX1276_MISO_PIN 24
    #define SX1276_MOSI_PIN 23
    #define SX1276_NSS_PIN 22
    #define SX1276_DIO0_PIN 13
    #define SX1276_DIO1_PIN 14
    #define SX1276_DIO2_PIN 15
    #define SX1276_DIO3_PIN 16
    #define SX1276_DIO4_A_PIN 19
    #define SX1276_DIO4_B_PIN 29

    // RX/TX pins for the GPS UART
    #define GPS_RX_PIN 17
    #define GPS_TX_PIN 18
    #define GPS_CTS_PIN -1
    #define GPS_RTS_PIN -1

    // SPI pins for the IMU
    #define IMU_MISO_PIN 18
    #define IMU_MOSI_PIN 17
    #define IMU_SCK_PIN 16
    #define IMU_CS_M_PIN 19
    #define IMU_CS_AG_PIN 20


#endif

#endif

