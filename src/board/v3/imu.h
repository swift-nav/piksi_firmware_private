/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swiftnav.com>
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

void imu_init(void);

static const double BMI160_DT_LOOKUP[] = {0.04, 0.02, 0.01, 0.005};

#endif
