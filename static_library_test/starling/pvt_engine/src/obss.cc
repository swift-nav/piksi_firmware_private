/*
 * Copyright (C) 2014,2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <pvt_engine/obss.h>

/** Correct observations with satellite clock parameters
 *
 * \param n_channels Number of tracking channel measurements
 * \param nav_meas Array of pointers of where to store the output observations,
 *                 length `n_channels`
 */
void apply_sat_clock_corrections(u8 n_channels,
                                 navigation_measurement_t *nav_meas[]) {
  for (u8 i = 0; i < n_channels; i++) {
    /* Correct the measurements with the satellite clock(rate) errors */
    double carrier_freq = sid_to_carr_freq(nav_meas[i]->sid);
    nav_meas[i]->pseudorange += GPS_C * nav_meas[i]->sat_clock_err;
    nav_meas[i]->carrier_phase += nav_meas[i]->sat_clock_err * carrier_freq;
    nav_meas[i]->measured_doppler +=
        nav_meas[i]->sat_clock_err_rate * carrier_freq;
  }
}

/** \} */
