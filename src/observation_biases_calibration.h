/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
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
#include <assert.h>
#include "base_obs.h"
#include <libswiftnav/common.h>
#include <libswiftnav/glo_map.h>
#include "sbp.h"
#include "sbp_utils.h"
#include "timing.h"
#include <libswiftnav/track.h>

/* Empirical corrections for GLO per-frequency pseudorange bias as per
 * https://github.com/swift-nav/piksi_v3_bug_tracking/issues/606#issuecomment-323163617
 * to align observations to Septentrio receiver-type biases
 */
static const double glo_l1_isc[] = {[0] = -7.25,
        [1] = -7.37,
        [2] = -7.5,
        [3] = -7.57,
        [4] = -7.51,
        [5] = -7.25,
        [6] = -7,
        [7] = -6.72,
        [8] = -7,
        [9] = -7.3,
        [10] = -7.73,
        [11] = -8.45,
        [12] = -8.95,
        [13] = -9.5};

static const double glo_l2_isc[] = {[0] = -7.5,
        [1] = -7.26,
        [2] = -6.83,
        [3] = -6.45,
        [4] = -6.27,
        [5] = -6.16,
        [6] = -6,
        [7] = -5.8,
        [8] = -5.5,
        [9] = -5.35,
        [10] = -5.25,
        [11] = -5.0,
        [12] = -5.0,
        [13] = -5.0};

static const double gps_l2_isc = -1.95;

/* These biases are to align the GLONASS carrier phase to the Septentrio
 * receivers carrier phase These biases are in cycles and are proportional to
 * the frequency number
 * */
static const double glo_l1_carrier_phase_bias = -0.07 / 8;
static const double glo_l2_carrier_phase_bias = 0;

/* This following constants describes the biases that will be sent through
 * SBP_MSG_GLO_BIASES in the sbp stream. Biases are to be expressed in meters
 * and are not quantized
 */
static const u8 broadcast_mask = 255;
static const double broadcast_l1ca_bias = 0.;
static const double broadcast_l1p_bias = 0.;
static const double broadcast_l2ca_bias = 0.;
static const double broadcast_l2p_bias = 0.;

void apply_isc_table(u8 n_channels,
                            navigation_measurement_t *nav_meas[]);

static const double biases_message_freq_setting = 1.0;
bool decimate_glonass_biases(const gps_time_t *_t);
void send_glonass_biases(void);

#endif //PIKSI_FIRMWARE_PRIVATE_OBSERVATION_BIASES_CALIBRATION_H