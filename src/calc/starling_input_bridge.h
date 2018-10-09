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

/**
 * Starling Input Bridge
 * =====================
 * This is a two-sided module which provides a thread-safe mechanism
 * for pushing data into the Starling engine. A sequence of "send" functions
 * allow a user to transmit data to Starling from any context.
 *
 * Symmetric "read" functions are then provided to the Starling engine
 * so it may go about its business.
 *
 * The API will operate in bypass mode until the bridge has been
 * initialized.
 */

/* Values returned by the firmware-facing functions. */
#define STARLING_SEND_OK 0
#define STARLING_SEND_ERROR 1

/* Operational modes for the Starling input bridge. */
#define STARLING_BRIDGE_MODE_DEFAULT 0
#define STARLING_BRIDGE_MODE_BYPASS 1

void starling_input_bridge_init(void);

void starling_input_bridge_set_mode(int mode);

/*******************************************************************************
 * Client-Facing "Send" Functions
 ******************************************************************************/

int starling_send_rover_obs(const gps_time_t *t,
                            const navigation_measurement_t *nm,
                            size_t n);

int starling_send_base_obs(const obs_array_t *obs_array);

int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n);

int starling_send_sbas_data(const sbas_raw_data_t *sbas_data);

int starling_send_imu_data(const imu_data_t *imu_data);

/*******************************************************************************
 * Starling-Facing "Read" Functions
 ******************************************************************************/

/* This side faces the Starling engine. */
void starling_wait(void);

int starling_receive_rover_obs(int blocking, obs_array_t *obs_array);

int starling_receive_base_obs(int blocking, obs_array_t *obs_array);

int starling_receive_ephemeris_array(int blocking,
                                     ephemeris_array_t *eph_array);

int starling_receive_sbas_data(int blocking, sbas_raw_data_t *sbas_data);

int starling_receive_imu_data(int blocking, imu_data_t *imu_data);

#endif
