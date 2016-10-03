#include <stdbool.h>

#include <nrf_drv_twi.h>

#include "app_error.h"

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

#include "oslmic.h"

#include "nrf52_pins.h"
#include "gps_job.h"
#include "battery_job.h"
#include "imu_job.h"
#include "lora_job.h"
#include "power_job.h"

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    NRF_LOG_INIT();
    NRF_LOG_PRINTF("Start main loop. Device ID = %04x%04x\n", NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);

    // Turn on various units through power-switch
    powerup_init();

    /* really lmic_init */
    os_init();

    lora_job_init();

    gps_job_init();

    battery_job_init();

    imu_job_init();

    os_runloop();

    NRF_LOG("You are not supposed to see this! os_runloop() shouldn't return!\n");
}

