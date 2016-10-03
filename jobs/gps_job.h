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

/**!brief Initialize the GPS job via the LMiC scheduler */
#ifndef GPS_JOB_H
#define GPS_JOB_H

/**
 * Interval for GPS fix
 */
#define GPS_FIX_INTERVAL 10

#define GPS_FIX_TIMEOUT (GPS_FIX_INTERVAL/2)
#define GPS_WARMUP_TIME 90


/**
 * Processing interval when waiting for fix
 */
#define GPS_PROCESS_INTERVAL 1

/**
 * Initialize the GPS receiver and start the fix checking job.
 */
void gps_job_init(void);

#endif
