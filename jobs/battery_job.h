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
 * The battery reading job. Reads the battery status every
 * BATTERY_UPDATE_INTERVAL seconds and updates the internal cache with the
 * value.
 */
#ifndef BATTERY_JOB_H
#define BATTERY_JOB_H

/**
 * Read interval for battery. Set to something lower if you want more
 * samples.
 */
#define BATTERY_UPDATE_INTERVAL 300

/**
 * Initialize SAADC and start battery reading job.
 */
void battery_job_init(void);

#endif
