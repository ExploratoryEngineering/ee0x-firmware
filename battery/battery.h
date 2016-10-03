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

#ifndef BATTERY_H
#define BATTERY_H
/**
 * @brief Start det SAADC sampler for battery tests.
 */
void battery_init();

/**
 * @brief Sample battery reading.
 *
 * This will eventually update the battery
 * reading that can be read with battery_get_mv().
 */
void battery_sample();

/**
 * @brief Read last battery value, in mV
 *
 * Get last reading from battery, in millivolts. Check the battery's
 * data sheet to determine an approximate percentage of battery level.
 */
void battery_get_mv(uint16_t* milli_volts);

/**
 * @brief Convert millivolts to a value 1-254 (for LoRa)
 *
 * 0 is external power source
 * 1 is minimum
 * 254 is maximum
 * 255 is unavailable
 */
uint8_t battery_to_byte(const uint16_t milli_volts);

/**
 * The discharge curve for the battery isn't known (according to the
 * data sheet for ER17505 it is flat until almost empty, then it drops
 * steeply) so we'll just use a simple linear curve to determine the
 * battery; 1 is empty (3.0V), 255 is max (3.7V or more)
 */
#define MAX_VOLTAGE 3700
#define MIN_VOLTAGE 3000

#endif
