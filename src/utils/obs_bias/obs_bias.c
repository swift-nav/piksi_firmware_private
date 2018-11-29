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
#include "obs_bias.h"
#include <assert.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/glonass_phase_biases.h>
#include "sbp/sbp_utils.h"

/* exported setting */
const u32 biases_message_freq_setting = 5.0;

/* Empirical corrections for GLO per-frequency pseudorange bias as per
 * https://github.com/swift-nav/piksi_v3_bug_tracking/issues/606#issuecomment-323163617
 * to align observations to Septentrio receiver-type biases
 */
static const double glo_l1_isc[] = {[0] = -9.4,
                                    [1] = -8.7,
                                    [2] = -9.0,
                                    [3] = -7.9,
                                    [4] = -8.6,
                                    [5] = -10.2,
                                    [6] = -8.3,
                                    [7] = -9.4,
                                    [8] = -9.2,
                                    [9] = -9.7,
                                    [10] = -10.8,
                                    [11] = -10.1,
                                    [12] = -10.5,
                                    [13] = -12.1};

static const double glo_l2_isc[] = {[0] = -5.9,
                                    [1] = -5.3,
                                    [2] = -5.0,
                                    [3] = -2.9,
                                    [4] = -4.5,
                                    [5] = -6.1,
                                    [6] = -3.9,
                                    [7] = -4.9,
                                    [8] = -4.1,
                                    [9] = -6.8,
                                    [10] = -5.4,
                                    [11] = -5.4,
                                    [12] = -5.4,
                                    [13] = -7.0};

/* TODO: estimate these properly against e.g. Septentrio  */
static const double gps_l2_isc = 1.9;
static const double bds2_b11_isc = -8.2;
static const double bds2_b2_isc = 0.1;
static const double gal_e1b_isc = -1.6;
static const double gal_e7i_isc = 6.4;

/* These biases are to align the GLONASS carrier phase to the Septentrio
 * receivers carrier phase These biases are in cycles and are proportional to
 * the frequency number
 * */
static const double glo_l1_carrier_phase_bias = -0.066;
static const double glo_l2_carrier_phase_bias = -0.042;

/** Apply ISC corrections from hard-coded table
 * Alignment is performed relative to the Septentrio
 * receiver type
 */
void apply_isc_table(obs_array_t *obs_array) {
  for (u8 i = 0; i < obs_array->n; i++) {
    double pseudorange_corr = 0;
    double carrier_phase_corr = 0;
    starling_obs_t *obs = &obs_array->observations[i];
    switch ((s8)obs->sid.code) {
      case CODE_GPS_L1CA:
        break;

      case CODE_GPS_L2CM:
        pseudorange_corr = gps_l2_isc;
        break;

      case CODE_GLO_L1OF:
        pseudorange_corr =
            0 * glo_l1_isc[glo_map_get_fcn(obs->sid) - GLO_MIN_FCN];
        carrier_phase_corr = (glo_map_get_fcn(obs->sid) - GLO_MIN_FCN) *
                             glo_l1_carrier_phase_bias;
        break;

      case CODE_GLO_L2OF:
        pseudorange_corr =
            0 * glo_l2_isc[glo_map_get_fcn(obs->sid) - GLO_MIN_FCN];
        carrier_phase_corr = (glo_map_get_fcn(obs->sid) - GLO_MIN_FCN) *
                             glo_l2_carrier_phase_bias;
        break;

      case CODE_BDS2_B1:
        pseudorange_corr = bds2_b11_isc;
        break;

      case CODE_BDS2_B2:
        pseudorange_corr = bds2_b2_isc;
        break;

      case CODE_GAL_E1B:
        pseudorange_corr = gal_e1b_isc;
        break;

      case CODE_GAL_E7I:
        pseudorange_corr = gal_e7i_isc;
        break;

      case CODE_INVALID:
      case CODE_COUNT:
        assert(!"Invalid code.");
        break;

      default:
        /* If code not supported we just return a zero correction. */
        break;
    }

    obs->pseudorange += pseudorange_corr;
    obs->carrier_phase += carrier_phase_corr;
  }
}

void send_glonass_biases(void) {
  static u8 buff[256];

  sbp_pack_glonass_biases_content(piksi_glonass_biases,
                                  (msg_glo_biases_t *)buff);
  sbp_send_msg(SBP_MSG_GLO_BIASES, sizeof(msg_glo_biases_t), buff);
}
