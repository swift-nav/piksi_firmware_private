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
#include "observation_biases_calibration.h"
#include "sbp_utils.h"

/** Apply ISC corrections from hard-coded table
 * Alignment is performed relative to the Septentrio
 * receiver type
 */
void apply_isc_table(u8 n_channels, navigation_measurement_t *nav_meas[]) {
  for (u8 i = 0; i < n_channels; i++) {
    double pseudorange_corr = 0;
    double carrier_phase_corr = 0;
    switch (nav_meas[i]->sid.code) {
      case CODE_GPS_L1CA:
        break;

      case CODE_GPS_L2CL:
      case CODE_GPS_L2CM:
        pseudorange_corr = gps_l2_isc;
        break;

      case CODE_GLO_L1OF:
        pseudorange_corr =
            glo_l1_isc[glo_map_get_fcn(nav_meas[i]->sid) - GLO_MIN_FCN];
        carrier_phase_corr = (glo_map_get_fcn(nav_meas[i]->sid) - GLO_MIN_FCN) *
                             glo_l1_carrier_phase_bias;
        break;

      case CODE_GLO_L2OF:
        pseudorange_corr =
            glo_l2_isc[glo_map_get_fcn(nav_meas[i]->sid) - GLO_MIN_FCN];
        carrier_phase_corr = (glo_map_get_fcn(nav_meas[i]->sid) - GLO_MIN_FCN) *
                             glo_l2_carrier_phase_bias;
        break;

      case CODE_INVALID:
      case CODE_COUNT:
        assert(!"Invalid code.");
        break;

      case CODE_SBAS_L1CA:
      case CODE_GPS_L1P:
      case CODE_GPS_L2P:
      case CODE_GPS_L2CX:
      case CODE_GPS_L5I:
      case CODE_GPS_L5Q:
      case CODE_GPS_L5X:
      case CODE_BDS2_B11:
      case CODE_BDS2_B2:
      case CODE_GAL_E1B:
      case CODE_GAL_E1C:
      case CODE_GAL_E1X:
      case CODE_GAL_E6B:
      case CODE_GAL_E6C:
      case CODE_GAL_E6X:
      case CODE_GAL_E7I:
      case CODE_GAL_E7Q:
      case CODE_GAL_E7X:
      case CODE_GAL_E8:
      case CODE_GAL_E5I:
      case CODE_GAL_E5Q:
      case CODE_GAL_E5X:
      case CODE_QZS_L1CA:
      case CODE_QZS_L2CM:
      case CODE_QZS_L2CL:
      case CODE_QZS_L2CX:
      case CODE_QZS_L5I:
      case CODE_QZS_L5Q:
      case CODE_QZS_L5X:
      default:
        /* If code not supported we just return a zero correction. */
        break;
    }

    nav_meas[i]->pseudorange += pseudorange_corr;
    nav_meas[i]->raw_pseudorange += pseudorange_corr;
    nav_meas[i]->raw_carrier_phase -= carrier_phase_corr;
  }
}

void send_glonass_biases(void) {
  static u8 buff[256];

  pack_glonass_biases_content(piksi_glonass_biases, (msg_glo_biases_t *)buff);
  sbp_send_msg(SBP_MSG_GLO_BIASES, sizeof(msg_glo_biases_t), buff);
}
