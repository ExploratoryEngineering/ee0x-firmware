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

#include "app_error.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include "nrf_drv_uart.h"

#include "nmealib.h"
#include "gps_std.h"
#include "firefly_x1.h"
#include "data_cache.h"
#include "nrf52_pins.h"
#include "tps22994.h"

/**
 * Dump NMEA sentences read from the receiver to the debug log
 */
#define DUMP_NMEA_INPUT 0

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

static uint8_t nmea_buffer[MAX_NMEA_BUFFER];
static uint8_t nmea_pos = 0;

static void read_data(void) {
    uint8_t data;
    if (app_uart_get(&data) == NRF_SUCCESS) {
        #if DUMP_NMEA_INPUT
        NRF_LOG_PRINTF("%c", data);
        #endif
        nmea_buffer[nmea_pos++] = data;
        if (nmea_pos >= MAX_NMEA_BUFFER) {
            nmea_pos = 0;
            return;
        }
        if (data == '\n') {
            nmea_buffer[nmea_pos] = 0;
            nmea_sentence_t sentence;
            if (nmea_parse(nmea_buffer, &sentence)) {
                if (sentence.type[0] == 'G' && sentence.type[2] == 'A') {
                    if (sentence.type[1] == 'G') {
                        // a very convoluted way of saying 'GGA'
                        gps_gga_t gga;
                        nmea_decode_gga(&sentence, &gga);
                        gps_update_gga(&gga);
                    }
                    else if (sentence.type[1] == 'S') {
                        // a very convoluted way of saying 'GSA'
                        gps_gsa_t gsa;
                        nmea_decode_gsa(&sentence, &gsa);
                        gps_update_gsa(&gsa);
                    }
                }
            }
            nmea_pos = 0;
            memset(nmea_buffer, 0, MAX_NMEA_BUFFER);
        }
    }
}

/**
 * UART events. Only the APP_UART_DATA_READY event is of interest
 */
static void uart_event_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_PRINTF("Communication error on UART: %d\n", p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            NRF_LOG("FIFO error on UART\n");
            break;

        case APP_UART_DATA_READY:
            read_data();
            break;

        case APP_UART_DATA:
            break;

        case APP_UART_TX_EMPTY:
            break;
    }
}

/**
 * Initialise library. Sets up UART and internal event handling.
 */
void gps_init(const app_uart_comm_params_t* uart_params) {
    uint32_t err_code = 0;

    #if DUMP_NMEA_INPUT
    NRF_LOG("Will dump NMEA in debug log\n");
    #endif

    APP_UART_FIFO_INIT(uart_params,
        UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
        uart_event_handler, APP_IRQ_PRIORITY_LOW, err_code);

    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(1000);
}

/**
 * Turn on power to GPS receiver
 */
void gps_wakeup() {
    tps22994_channel_on(TPS22994_I2C_ADDRESS, TPS22994_GPS_PIN);
}

/**
 * Turn off power to GPS receiver
 */
void gps_sleep() {
    tps22994_channel_off(TPS22994_I2C_ADDRESS, TPS22994_GPS_PIN);
}

