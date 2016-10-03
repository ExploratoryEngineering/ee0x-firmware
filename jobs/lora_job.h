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

/**
 * LoRa setup and sending. The process itself is very simple and runs in a set
 * loop sending the data cached by the other jobs. Rate is pretty aggressive wrt
 * duty cycle so it should be dialled back a few notches for the final version.
 *
 * In general: Code is quite messy and is more proof-of-concept.
 */
#ifndef LORA_H
#define LORA_H

/**
 * Print status messages
 */
#define DUMP_STATUS 1

/**
 * Send message every N seconds.
 */
#define SEND_RATE 30

/**
 * Dump radio and LoRa status every N seconds
 */
#define STATUS_RATE 30


/**
 * LoRa network ID. ID is 0 for loriot.io (free account)
 */
#define LORAWAN_NET_ID 0x00000000

/**
 * Message ack on/off
 */
#define LORAWAN_CONFIRMED_MSG_ON 1

/**
 * Port (in LoRa message)
 */
#define LORAWAN_APP_PORT 13

/**
 * ADR for client on/off
 */
#define LORAWAN_ADR_ON 1

#define LORAWAN_OTAA 0

/**
 * Public network on/off
 */
#define LORAWAN_PUBLIC_NETWORK true

// Device provisioning - replace this with EUI/device address and NwkSKey + AppSKey as required.
#define LORAWAN_DEVICE_EUI { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 }

// APB parameters
#define LORAWAN_DEVICE_ADDRESS (uint32_t) 0x01020304
#define LORAWAN_NWKSKEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }
#define LORAWAN_APPSKEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }

// OTAA parameters
#define LORAWAN_APP_KEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }
#define LORAWAN_APP_EUI { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 }
/**
 * Init and start the LoRa sending job.
 */
void lora_job_init(void);

#endif
