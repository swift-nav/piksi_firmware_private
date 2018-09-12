/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef OBSERVATION_BIASES_CALIBRATION_H
#define OBSERVATION_BIASES_CALIBRATION_H

#include <swiftnav/nav_meas.h>

/* Send the glonass bias message every 5 observations message */
extern const u32 biases_message_freq_setting;

void apply_isc_table(u8 n_channels, navigation_measurement_t *nav_meas[]);

void send_glonass_biases(void);

#endif  // PIKSI_FIRMWARE_PRIVATE_OBSERVATION_BIASES_CALIBRATION_H
