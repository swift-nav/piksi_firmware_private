/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jonathan Diamond <jonathan@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _SWIFTNAV_IMU_H_
#define _SWIFTNAV_IMU_H_

#include <stdbool.h>
#include <libsbp/common.h>

/*
    Code for interacting with BMI160 IMU and BMM150 magnetometer
*/

void imu_init(void);

void imu_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag);

void imu_get_data(s16 acc[static 3], s16 gyro[static 3], s16 mag[static 3], u32* sensor_time);

u8 imu_read_reg(u8 reg_addr);

void imu_write_reg(u8 reg_addr, u8 value);

//read id register and check if it matches expected value
bool imu_check_id(void);

//read id register and check if it matches expected value
bool mag_check_id(void);

u8 imu_read_status(void);

u8 imu_read_error(void);

u8 imu_read_pmu_status(void);

#endif
