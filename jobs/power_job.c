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

#include <nrf_drv_twi.h>
#include <app_error.h>
#include "power_job.h"
#include "tps22994.h"

static nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);

void powerup_init(void) {
    tps22994_init(TPS22994_I2C_ADDRESS, &twi);

    tps22994_channel_on(TPS22994_I2C_ADDRESS, TPS22994_GPS_PIN);
    tps22994_channel_on(TPS22994_I2C_ADDRESS, TPS22994_VBACKUP_PIN);
    tps22994_channel_on(TPS22994_I2C_ADDRESS, TPS22994_BNO055_IO_PIN);
    tps22994_channel_on(TPS22994_I2C_ADDRESS, TPS22994_BNO055_PIN);
 }