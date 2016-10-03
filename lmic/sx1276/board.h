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

#ifndef _BOARD_H
#define _BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "oslmic.h"
#include "nrf52_pins.h"
/*
 * Some quick declarations and wrappers to make the driver easier to port
 */

#define Gpio_t uint8_t
typedef struct Spi_s {
	uint8_t Nss;
} Spi_t;

void GpioWrite(const Gpio_t* pin, const uint8_t high);
uint8_t SpiInOut(const Spi_t* spi, uint8_t data);

typedef struct TimerEvent_s {
	uint8_t timer_id;
	uint32_t ticks;
	bool triggered;
} TimerEvent_t;

typedef void(* TimerEventHandler_t)(void);

void TimerInit(TimerEvent_t* event, const TimerEventHandler_t event_handler);
void TimerStop(const TimerEvent_t* event);
void TimerSetValue(TimerEvent_t* event, const uint32_t ticks);
void TimerStart(const TimerEvent_t* event);
void DelayMs(const uint32_t ms);

#define memcpy1 memcpy

#endif
