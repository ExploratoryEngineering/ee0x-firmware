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

/**
 * Hardware setup and interface for Firefly X1 receiver. Not a lot of custom
 * code (just a single NMEA sentence to put the chip into sleep mode). Uses the
 * UART.
 */
#include "app_uart.h"

#define PMTK161 "$PMTK161,0*28\r\n"
#define PMTK400 "$PMTK400*36\r\n"

/**
 * Initialise library. Sets up UART and internal event handling.
 */
void gps_init(const app_uart_comm_params_t* uart_params);

/**
 * Wake up receiver.
 */
void gps_wakeup();

/**
 * Put receiver into sleep mode.
 */
void gps_sleep();


