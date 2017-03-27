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

#include "display_job.h"
#include "display.h"

/**
 * Job handle for the init job.
 */
static osjob_t displayjob;

/**
 * iaq sensor update function.
 */


static void display_func(osjob_t* job) {
    NRF_LOG_PRINTF("display_func..\n");

    DisplayTextBuffer();

    // Latest sample data can be retrieved by calling GetSensorReading()
    // Example: iAQSensor_t iaq = GetSensorReading();
    os_setTimedCallback(job, os_getTime() + sec2osticks(DISPLAY_UPDATE_INTERVAL), display_func);
}

void display_job_init(void) {
   	NRF_LOG_PRINTF("display_job_init..\n");
    TestDisplay();
  os_setTimedCallback(&displayjob, os_getTime() + sec2osticks(DISPLAY_UPDATE_INTERVAL), display_func);
}
