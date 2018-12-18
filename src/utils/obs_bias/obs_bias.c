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

/* Piksi Multi HW inter-signal biases calibrated against Skydel / Septentrio  */
static const double gps_l2_isc = -1.3;
static const double glo_l1_isc = -7.4;
static const double glo_l2_isc = -0.4;
static const double bds2_b11_isc = 0.3;
static const double bds2_b2_isc = 3.6;
static const double gal_e1b_isc = -0.2;
static const double gal_e7i_isc = 3.3;

/* These biases are to align the GLONASS carrier phase to the Septentrio
 * receivers carrier phase These biases are in cycles and are proportional to
 * the frequency number
 * */
static const double glo_l1_carrier_phase_bias = -0.070;
static const double glo_l2_carrier_phase_bias = -0.047;

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
        pseudorange_corr = glo_l1_isc;
        carrier_phase_corr = (glo_map_get_fcn(obs->sid) - GLO_MIN_FCN) *
                             glo_l1_carrier_phase_bias;
        break;

      case CODE_GLO_L2OF:
        pseudorange_corr = glo_l2_isc;
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
