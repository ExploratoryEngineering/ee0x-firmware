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

#include "lora_job.h"
#include "lora_config.h"
#include "lmic.h"
#include "app_error.h"

#define PORTNO 1

#if !LORAWAN_OTAA
static u1_t NWSKEY[16] =  LORAWAN_NWKSKEY;
static u1_t APPSKEY[16] = LORAWAN_APPSKEY;
#endif


static osjob_t sendjob;
static osjob_t initjob;

static void send_func(osjob_t* job) {
    uint8_t len = 0;
    LMIC.frame[len++] = (LMIC.rssi & 0xFF00 >> 8);
    LMIC.frame[len++] = (LMIC.rssi & 0x00FF);
    LMIC.frame[len++] = LMIC.snr;

    NRF_LOG("LoRa: Sending message...\n");
    if (LMIC_setTxData2( PORTNO, LMIC.frame, len, LORAWAN_CONFIRMED_MSG_ON ) != 0) {
        NRF_LOG("LoRa: *** Error sending frame!\n");
    } else {
        NRF_LOG_PRINTF("LoRa: %d bytes scheduled for output\n", len);
    }
}

void onEvent (ev_t ev) {
    switch (ev) {
        case EV_SCAN_TIMEOUT:   NRF_LOG("LoRa: SCAN_TIMEOUT\n"); break;
        case EV_BEACON_FOUND:   NRF_LOG("LoRa: BEACON_FOUND\n"); break;
        case EV_BEACON_MISSED:  NRF_LOG("LoRa: BEACON_MISSED\n"); break;
        case EV_BEACON_TRACKED: NRF_LOG("LoRa: BEACON_TRACKED\n"); break;
        case EV_JOINING:        NRF_LOG("LoRa: JOINING\n"); break;
        case EV_JOINED:         NRF_LOG("LoRa: JOINED\n");
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(SEND_RATE), send_func);
            break;
        case EV_RFU1:           NRF_LOG("LoRa: RFU1\n"); break;
        case EV_JOIN_FAILED:    NRF_LOG("LoRa: JOIN_FAILED\n"); break;
        case EV_REJOIN_FAILED:  NRF_LOG("LoRa: REJOIN_FAILED\n"); break;
        case EV_TXCOMPLETE:     NRF_LOG("LoRa: TXCOMPLETE\n");
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(SEND_RATE), send_func);
            break;
        case EV_LOST_TSYNC:     NRF_LOG("LoRa: LOST_TSYNC\n"); break;
        case EV_RESET:          NRF_LOG("LoRa: RESET\n"); break;
        case EV_RXCOMPLETE:     NRF_LOG("LoRa: RXCOMPLETE\n"); break;
        case EV_LINK_DEAD:      NRF_LOG("LoRa: LINK_DEAD\n"); break;
        default:
            NRF_LOG_PRINTF("LoRa: Uknown event: %d\n", ev);
            break;
    }
}

static void initfunc (osjob_t* job) {
    LMIC_reset();
    LMIC_setAdrMode( LORAWAN_ADR_ON );
    NRF_LOG("LoRa: Init ");
    #if LORAWAN_OTAA
        NRF_LOG("...using OTAA\n");
        LMIC_startJoining();
    #else
        NRF_LOG("...using ABP\n");
        LMIC_setSession(LORAWAN_NET_ID, LORAWAN_DEVICE_ADDRESS, NWSKEY, APPSKEY);
    #endif
}

void lora_job_init(void) {
    os_setCallback(&initjob, initfunc);
}
