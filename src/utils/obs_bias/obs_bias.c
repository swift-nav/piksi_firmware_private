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

/** Inter-signal corrections
 *
 * These hard-coded constants calibrate the Piksi Multi observation outputs to
 * those of the Septentrio receiver. Calibration was done in Dec 2018 against
 * the live-roof-650-townsend-msm7 scenario, using just the zero-baseline boards
 * of the pairs
 * */

/* clang-format off */

/* pseudorange biases (meters) */
static const double gps_l2_isc = -1.1;
static const double bds2_b11_isc = 0.4;
static const double bds2_b2_isc = 3.6;
static const double gal_e1b_isc = -0.2;
static const double gal_e7i_isc = 3.4;

/* GLO pseudorange biases per FCN (meters) */
static const double glo_l1_isc[GLO_MAX_FCN] =
  {-7.6,-7.5,-7.3,-7.2,-7.0,-7.1,-6.9,-6.9,-7.0,-6.9,-7.1,-7.5,-7.5,-7.9};
static const double glo_l2_isc[GLO_MAX_FCN] =
  {-2.9,-2.3,-1.8,-1.2,-1.2,-1.2,-0.8,+0.0,+0.2,-0.2,+0.8,+0.8,+0.9,+1.0};

/* GLO carrier biases per FCN (cycles) */
/* TODO: add a second set for Piksi Base version */
static const double glo_l1_carrier_corr = -0.017;
static const double glo_l2_carrier_corr = -0.007;

/* clang-format on */

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

      case CODE_GLO_L1OF: {
        u8 fcn_index = glo_map_get_fcn(obs->sid) - GLO_MIN_FCN;
        pseudorange_corr = glo_l1_isc[fcn_index];
        carrier_phase_corr = glo_l1_carrier_corr * fcn_index;
        break;
      }

      case CODE_GLO_L2OF: {
        u8 fcn_index = glo_map_get_fcn(obs->sid) - GLO_MIN_FCN;
        pseudorange_corr = glo_l2_isc[fcn_index];
        carrier_phase_corr = glo_l2_carrier_corr * fcn_index;
        break;
      }

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
