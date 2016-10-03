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

#include "gps_job.h"
#include "app_error.h"
#include "nrf_drv_uart.h"
#include "app_uart.h"

#include "oslmic.h"

#include "firefly_x1.h"
#include "data_cache.h"
#include "nrf52_pins.h"

/**
 * States for the internal state machine
 */
typedef enum {
    UNINITIALIZED,
    SLEEPING,
    WAITING_FOR_FIX,
    GOT_FIX,
    FIX_TIMEOUT
} gps_state_t;

/**
 * Internal state.
 */
static gps_state_t gps_state = UNINITIALIZED;

/**
 * Counter for time spent waiting for fix.
 */
static uint8_t wait_time = 0;

/**
 * The current timeout value. The initial value is different from the
 * general timeout since the GPS receiver might need some extra time to
 * acquire a fix.
 */
static uint8_t max_timeout = GPS_WARMUP_TIME;

/**
 * The GPS job handle.
 */
static osjob_t gpsjob;

/**
 * Start the UART interface
 */
static void start_gps() {
    const app_uart_comm_params_t comm_params = {
          .rx_pin_no = GPS_RX_PIN,
          .tx_pin_no = GPS_TX_PIN,
          .rts_pin_no = GPS_RTS_PIN,
          .cts_pin_no = GPS_CTS_PIN,
          .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
          .use_parity = false,
          .baud_rate = UART_BAUDRATE_BAUDRATE_Baud9600
      };

    gps_init(&comm_params);
}

/**
 * GPS handling. Since the LMiC library dictates how ALL THE OTHER code
 * must run we have to make a state machine for the GPS processing. Not too
 * bad but quite verbose.
 */
static void gps_func(osjob_t* job) {
    gps_fix_t fix;

    switch (gps_state) {
        case UNINITIALIZED:
            start_gps();
            gps_state = WAITING_FOR_FIX;
            max_timeout = GPS_WARMUP_TIME;
            // The GPS receiver might be in sleep mode here if the process has
            // been restarted. Wake it up just to be sure.
            gps_wakeup();

            // Schedule new check in 1 second
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_PROCESS_INTERVAL), gps_func);
            break;

        case SLEEPING:
            gps_reset_cache();
            gps_wakeup();
            wait_time = 0;
            gps_state = WAITING_FOR_FIX;
            max_timeout = GPS_FIX_INTERVAL;
            // Schedule new check in 1 second
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_PROCESS_INTERVAL), gps_func);
            break;

        case WAITING_FOR_FIX:
            wait_time++;
            if (gps_get_fix(&fix)) {
                gps_state = GOT_FIX;
            }
            if (wait_time > max_timeout) {
                gps_state = FIX_TIMEOUT;
            }
            // Schedule in 1 second. A bit wasteful if we've got fix or a timeout but...
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_PROCESS_INTERVAL), gps_func);
            break;

        case FIX_TIMEOUT:
            NRF_LOG_PRINTF("GPS: Fix timeout after %d seconds\n", wait_time);
            gps_add_timeout_stat();
            gps_sleep();
            gps_state = SLEEPING;
            // Schedule wake up in GPS_FIX_INTERVAL seconds
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_FIX_INTERVAL), gps_func);
            break;

        case GOT_FIX:
            NRF_LOG_PRINTF("GPS: Fix after %d seconds\n", wait_time);
            gps_add_fix_stat(wait_time);
            gps_sleep();
            gps_state = SLEEPING;
            // Schedule wake up in GPS_FIX_INTERVAL seconds
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_FIX_INTERVAL), gps_func);
            break;

        default:
            // Something is very very wrong
            NRF_LOG("GPS: Don't know how I got into this state, Changing to SLEEPING\n");
            gps_state = SLEEPING;
            os_setTimedCallback(job, os_getTime() + sec2osticks(GPS_PROCESS_INTERVAL), gps_func);
            break;
    }
}

void gps_job_init(void) {
    os_setTimedCallback(&gpsjob, os_getTime(), gps_func);
}
