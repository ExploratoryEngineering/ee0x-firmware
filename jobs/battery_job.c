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

#include "app_error.h"
#include "oslmic.h"

#include "battery_job.h"
#include "battery.h"

/**
 * Job handle for the init job.
 */
static osjob_t batteryjob;

/**
 * Battery update function.
 */
static void battery_func(osjob_t* job) {
    battery_sample();
    uint16_t mv = 0;
    battery_get_mv(&mv);
    NRF_LOG_PRINTF("Read battery: %d mV\n", mv);
    os_setTimedCallback(job, os_getTime() + sec2osticks(BATTERY_UPDATE_INTERVAL), battery_func);
}

void battery_job_init(void) {
  battery_init();

  os_setTimedCallback(&batteryjob, os_getTime() + sec2osticks(5), battery_func);
}
