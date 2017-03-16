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


/**
 * Public network on/off
 */
#define LORAWAN_PUBLIC_NETWORK true


/**
 * Init and start the LoRa sending job.
 */
void lora_job_init(void);

#endif
