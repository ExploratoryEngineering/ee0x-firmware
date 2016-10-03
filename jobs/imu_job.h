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

#ifndef IMU_JOB_H
#define IMU_JOB_H

/**
 * Read inteval for IMU.
 */
#define IMU_INTERVAL 30

// I2C addresses for magnetometer + accelerometer/gyro
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW - magnetometer address
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW - accel/gyro address

/**
 * Initialize and start the IMU reading job.
 */
void imu_job_init(void);

#endif
