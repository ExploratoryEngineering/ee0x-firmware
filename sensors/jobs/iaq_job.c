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

#include "iaq_job.h"
#include "co2-sensor.h"

static nrf_drv_twi_t iaq_twi = NRF_DRV_TWI_INSTANCE(0);

/**
 * Job handle for the init job.
 */
static osjob_t iaqjob;

/**
 * iaq sensor update function.
 */


static void iaq_func(osjob_t* job) {
    iAQCore_sampleCO2();
    // Latest sample data can be retrieved by calling GetSensorReading()
    // Example: iAQSensor_t iaq = GetSensorReading();
    os_setTimedCallback(job, os_getTime() + sec2osticks(IAQ_UPDATE_INTERVAL), iaq_func);
}

void iaq_job_init(void) {
  iAQCore_init(&iaq_twi); 
  os_setTimedCallback(&iaqjob, os_getTime() + sec2osticks(30), iaq_func);
}
