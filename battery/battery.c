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
#include "nrf_drv_saadc.h"
#include "app_util_platform.h"
#include "battery.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS       600  /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION        6    /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS      270  /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RES_10BIT                       1024 /**< Maximum digital value for 10-bit ADC conversion. */

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static nrf_saadc_value_t adc_buf[2];

static uint16_t current_battery_mv = 0;

/**
 *  @brief Update the battery reading
 */
static void battery_update_mv(const uint16_t mv) {
    CRITICAL_REGION_ENTER();
    current_battery_mv = mv;
    CRITICAL_REGION_EXIT();
}

/**
 * @brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {

        nrf_saadc_value_t adc_result = p_event->data.done.p_buffer[0];

        APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1));


        uint16_t batt_mv = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;

        battery_update_mv(batt_mv);
    }
}

void battery_init() {
    // Set up SAADC to use VDD input
    APP_ERROR_CHECK(nrf_drv_saadc_init(NULL, saadc_event_handler));
    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    APP_ERROR_CHECK(nrf_drv_saadc_channel_init(0,&config));
    APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(&adc_buf[0], 1));
    APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(&adc_buf[1], 1));
}

void battery_sample() {
    APP_ERROR_CHECK(nrf_drv_saadc_sample());
}

void battery_get_mv(uint16_t* milli_volts) {
    CRITICAL_REGION_ENTER();
    *milli_volts = current_battery_mv;
    CRITICAL_REGION_EXIT();
}


uint8_t battery_to_byte(const uint16_t milli_volts) {
    // TODO: Proper battery curve
    uint16_t remaining = milli_volts - MIN_VOLTAGE;
    float range = (MAX_VOLTAGE - MIN_VOLTAGE);
    float ratio = 254 * remaining/range;
    if (ratio < 1.0f) {
        return 1;
    }
    if (ratio > 254.0f) {
        return 254;
    }
    return (uint8_t) ratio;
}
