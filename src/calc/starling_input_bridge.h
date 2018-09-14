/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_INPUT_BRIDGE_H_
#define STARLING_INPUT_BRIDGE_H_

#include <starling/starling.h>

/* Values returned by the firmware-facing functions. */
#define STARLING_SEND_OK    0
#define STARLING_SEND_ERROR 1

/* This side faces the firmware. */
void starling_input_bridge_init(void);

int starling_send_rover_obs(const gps_time_t *t, 
                            const navigation_measurement_t *nm,
                            size_t n);

int starling_send_base_obs(const obs_array_t *obs_array);

int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n);

int starling_send_sbas_data(const sbas_raw_data_t *sbas_data);

/* This side faces the Starling engine. */
void starling_wait(void);

int starling_read_rover_obs(int blocking, obs_array_t *obs_array);

int starling_read_base_obs(int blocking, obs_array_t *obs_array);

int starling_read_ephemeris_array(int blocking, ephemeris_array_t *eph_array);

int starling_read_sbas_data(int blocking, sbas_raw_data_t *sbas_data);

#endif




