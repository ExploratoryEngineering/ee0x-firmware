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

#include <string.h>

#include <nrf_drv_twi.h>
#include <app_error.h>

#include "tps22994.h"

static nrf_drv_twi_t* twi;

void tps22994_init_i2c()
{
    /* Make sure I2C is configured and enabled */

    APP_ERROR_CHECK(nrf_drv_twi_init(twi, NULL, NULL, NULL));
    nrf_drv_twi_enable(twi);
}

void tps22994_init(uint8_t address, nrf_drv_twi_t* twidriver)
{
    twi = twidriver;

    tps22994_init_i2c();

    // Make sure the channels are controlled via I2C
    tps22994_write_ctrl_reg(address, 0xF0);
}

uint8_t tps22994_read_byte(uint8_t address, uint8_t reg)
{
    uint8_t data = 0;

    APP_ERROR_CHECK(nrf_drv_twi_tx(twi, address, &reg, sizeof(reg), false));
    APP_ERROR_CHECK(nrf_drv_twi_rx(twi, address, &data, 1));
    return data;
}

uint8_t tps22994_read_bytes(uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count)
{
    APP_ERROR_CHECK(nrf_drv_twi_tx(twi, address, &reg, sizeof(reg), false));
    APP_ERROR_CHECK(nrf_drv_twi_rx(twi, address, dest, count));
    return count;
}

void tps22994_write_byte(uint8_t address, uint8_t reg, uint8_t data)
{
    uint8_t all_data[] = { reg, data };
    APP_ERROR_CHECK(nrf_drv_twi_tx(twi, address, all_data, sizeof(all_data), false));
}

void tps22994_channel_off(uint8_t address, uint8_t channel)
{
    uint8_t ctrl_reg;

    ctrl_reg = tps22994_read_byte(address, TPS22994_REG_CONTROL);
    ctrl_reg &= ~(1 << (channel & 0xF) );
    tps22994_write_byte(address, TPS22994_REG_CONTROL, ctrl_reg);
}

void tps22994_channel_on(uint8_t address, uint8_t channel)
{
    uint8_t ctrl_reg;

    ctrl_reg = tps22994_read_byte(address, TPS22994_REG_CONTROL);
    ctrl_reg |= (1 << (channel & 0xF) );
    tps22994_write_byte(address, TPS22994_REG_CONTROL, ctrl_reg);
}

void tps22994_channel_toggle(uint8_t address, uint8_t channel)
{
    uint8_t ctrl_reg;

    ctrl_reg = tps22994_read_byte(address, TPS22994_REG_CONTROL);
    ctrl_reg ^= (1 << (channel & 0xF) );
    tps22994_write_byte(address, TPS22994_REG_CONTROL, ctrl_reg);
}


uint8_t tps22994_read_ctrl_reg(uint8_t address)
{
    uint8_t data;
    data = tps22994_read_byte(address, TPS22994_REG_CONTROL);
    return data;
}

uint8_t tps22994_read_reg(uint8_t address, uint8_t reg)
{
    uint8_t data;
    data = tps22994_read_byte(address, reg);
    return data;
}

void tps22994_write_ctrl_reg(uint8_t address, uint8_t value)
{
    tps22994_write_byte(address, TPS22994_REG_CONTROL, value);
}

void tps22994_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    tps22994_write_byte(address, reg, value);
}
