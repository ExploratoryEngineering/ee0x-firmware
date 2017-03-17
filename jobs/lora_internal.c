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

/* These are internal functions for the LMiC library. They provide the
configuration values that the device will use. */
#include "lora_job.h"
#include "lmic.h"
#include "lora_config.h"


#if !LORAWAN_OTAA
static u1_t NWSKEY[16] =  LORAWAN_NWKSKEY;
static u1_t APPSKEY[16] = LORAWAN_APPSKEY;
#endif

// application router ID (LSBF)
static const u1_t APPEUI[8]  = LORAWAN_APP_EUI;

// unique device ID (LSBF)
static const u1_t DEVEUI[8] = LORAWAN_DEVICE_EUI;

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = LORAWAN_APP_KEY;

// Note: Unused callbacks
// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    LMIC_reverse_memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    LMIC_reverse_memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes) q
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}
