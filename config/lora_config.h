/*
**   Copyright 2017 Telenor Digital AS
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
#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

/* ==========================================================================
 * OTAA provisioning. This includes the device EUI, application EUI and 
 * application key. The application and network session keys will be 
 * negotiated when the device joins the network.
 * ========================================================================== */
// Device provisioning - replace this with EUI/device address and NwkSKey + AppSKey as required.
#define LORAWAN_DEVICE_EUI { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 }
#define LORAWAN_APP_KEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }
#define LORAWAN_APP_EUI { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 }

/* ==========================================================================
 * ABP (activation by personalisation) parameters. Rather than negotiating 
 * the application and network session keys these are set directly. This 
 * eliminates the need for a join request and the device can transmit data 
 * directly. The downside is that the device should keep track of the frame
 * counters when it is powered down but you won't share the application key.
 * ========================================================================== */
#define LORAWAN_DEVICE_ADDRESS (uint32_t) 0x01020304
#define LORAWAN_NWKSKEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }
#define LORAWAN_APPSKEY { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }

#endif