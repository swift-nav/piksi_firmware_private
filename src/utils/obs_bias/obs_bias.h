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
#ifndef SWIFTNAV_OBSERVATION_BIASES_CALIBRATION_H
#define SWIFTNAV_OBSERVATION_BIASES_CALIBRATION_H

#include <starling/observation.h>
#include <swiftnav/nav_meas.h>

/* Send the glonass bias message every 5 observations message */
extern const u32 biases_message_freq_setting;

void apply_isc_table(obs_array_t *obs_array);

void send_glonass_biases(void);

#endif /* SWIFTNAV_OBSERVATION_BIASES_CALIBRATION_H */
