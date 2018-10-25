/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>

#include "calc_nav_meas.h"
#include "me_constants.h"
#include "nav_msg/cnav_msg_storage.h"
#include "track/track_sid_db.h"

#include <starling/cycle_slip.h>
#include <starling/starling.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/troposphere.h>

/* Convert a single channel measurement into a single navigation measurement. */
static s8 convert_channel_measurement_to_starling_obs(
    const gps_time_t *rec_time,
    const channel_measurement_t *meas,
    starling_obs_t *obs) {
  obs->sid = meas->sid;

  u32 code_length = code_to_chip_count(meas->sid.code);
  u32 chips_in_millisecond =
      code_length / code_to_prn_period_ms(meas->sid.code);
  double chip_rate = code_to_chip_rate(meas->sid.code);
  double lambda = sid_to_lambda(meas->sid);

  /* Compute the time of transmit of the signal on the satellite from the
   * tracking loop parameters. This will be used to compute the pseudorange.
   */
  obs->tot.wn = WN_UNKNOWN;
  obs->tot.tow = 1e-3 * meas->time_of_week_ms;
  double chips = meas->code_phase_chips;
  if (chips > code_length) {
    /* Sanity check of the code phase measurement */
    log_warn_sid(obs->sid, "Measured code phase exceeds code length");
    /* TODO: flag only this measurement's PR, CP and Doppler inaccurate
     * instead of terminating and effectively discarding all measurements */
    return -1;
  }

  /* Add the fractional millisecond part to the time of reception.
   * Because the code phase is from early correlator, measurement reporting
   * within the last chip of the PRN actually came from the next full
   * millisecond. */
  while (chips > chips_in_millisecond - 1) {
    /* Note the loop will run at most once for L1CA or 20 rounds for L2CM. */
    chips -= chips_in_millisecond;
  }
  obs->tot.tow += chips / chip_rate;

  obs->tot.tow += meas->tow_residual_ns * 1e-9;

  normalize_gps_time(&obs->tot);

  /* Match the week number to the time of reception. */
  gps_time_match_weeks(&obs->tot, rec_time);

  /* Compute the carrier phase measurement. */
  obs->carrier_phase = meas->carrier_phase;

  /* For raw Doppler we use the instantaneous carrier frequency from the
   * tracking loop. */
  obs->doppler = meas->carrier_freq;

  /* Copy over remaining values. */
  obs->cn0 = meas->cn0;
  obs->lock_time = meas->lock_time;

  /* Measurement time offset from rec_time, usually -5ms .. -0ms */
  double dt = meas->rec_time_delta;

  /* Form the time of reception of this signal */
  gps_time_t meas_tor = *rec_time;
  meas_tor.tow += dt;
  normalize_gps_time(&meas_tor);

  /* The raw pseudorange is just the time of flight multiplied by the speed of
   * light. */
  obs->pseudorange = GPS_C * (gpsdifftime(&meas_tor, &obs->tot));

  /* Finally, propagate measurement back to reference time */
  obs->tot.tow -= dt;
  normalize_gps_time(&obs->tot);

  /* Propagate pseudorange with raw doppler times wavelength */
  obs->pseudorange += dt * obs->doppler * lambda;
  /* Propagate carrier phase with carrier frequency */
  obs->carrier_phase += dt * obs->doppler;

  /* Compute flags.
   *
   * \note currently algorithm uses 1 to 1 flag mapping, however it can use
   *       channel measurement flags to compute errors and weight factors.
   */
  obs->flags = 0;
  if (0 != (meas->flags & CHAN_MEAS_FLAG_CODE_VALID)) {
    obs->flags |= NAV_MEAS_FLAG_CODE_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_PHASE_VALID)) {
    obs->flags |= NAV_MEAS_FLAG_PHASE_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    obs->flags |= NAV_MEAS_FLAG_MEAS_DOPPLER_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN)) {
    obs->flags |= NAV_MEAS_FLAG_HALF_CYCLE_KNOWN;
  }
  obs->flags |= NAV_MEAS_FLAG_CN0_VALID;

  return 0;
}

/** Calculate observations from tracking channel measurements.
 *
 * \param n_channels Number of tracking channel measurements
 * \param meas Array of tracking channel measurements, length `n_channels`
 * \param obs_array Observation array for storing the observations
 * \param rec_time Pointer to an estimate of the GPS time at reception time
 * \return '0' for success, '-1' for measurement sanity check error
 */
s8 calc_navigation_measurement(u8 n_channels,
                               const channel_measurement_t meas[],
                               obs_array_t *obs_array,
                               const gps_time_t *rec_time) {
  /* initialize the obs array */
  obs_array->sender = 0;
  obs_array->t = *rec_time;
  obs_array->n = n_channels;

  /* To calculate the pseudorange from the time of transmit we need the local
   * time of reception. */
  if (!gps_time_valid(rec_time)) {
    log_error("Invalid gps time in calc_navigation_measurement n_channels = %u",
              n_channels);
    return -1;
  }

  assert(n_channels <= MAX_CHANNELS);
  for (u8 i = 0; i < n_channels; ++i) {
    s8 ret = convert_channel_measurement_to_starling_obs(
        rec_time, &meas[i], &obs_array->observations[i]);
    if (ret) {
      return ret;
    }
  }

  return 0;
}

/** Calculate and ISC value for selected code if value is available.
 *  Return true or false upon success.
 *
 * See IS-GPS-200H 30.3.3.3.1.1.1
 *
 * \param code Code used
 * \param msg ISC data message
 * \param[out] isc pointer to the result
 * \return true if ISC was available from CNAV, false otherwise
 */
static bool get_isc_corr(const code_t code,
                         const cnav_msg_type_30_t *msg,
                         double *isc) {
  if (NULL == msg) {
    return false;
  }

  switch ((s8)code) {
    case CODE_GPS_L1CA:
      if (msg->tgd_valid && msg->isc_l1ca_valid) {
        *isc = (-msg->tgd + msg->isc_l1ca) * GROUP_DELAY_SCALE * GPS_C;
        return true;
      }
      break;

    case CODE_GPS_L2CM:
      if (msg->tgd_valid && msg->isc_l2c_valid) {
        *isc = (-msg->tgd + msg->isc_l2c) * GROUP_DELAY_SCALE * GPS_C;
        return true;
      }
      break;

    case CODE_INVALID:
    case CODE_COUNT:
      assert(!"Invalid code.");
      break;

    default:
      break;
  }

  return false;
}

/** Apply ISC corrections
 * This function applies ISC corrections to the measurements calculated by
 * calc_navigation_measurement().
 *
 * Note: the correction computed from CNAV is to be applied instead of the plain
 * TGD available in LNAV ephemeris. Thus this function removes the already
 * applied TGD from the measurements where CNAV is available.
 */
void apply_gps_cnav_isc(u8 n_channels,
                        navigation_measurement_t *nav_meas[],
                        const ephemeris_t ephe[]) {
  u8 i = 0;
  for (i = 0; i < n_channels; i++) {
    double isc;
    cnav_msg_t cnav_msg;
    /* get GPS inter-signal corrections from CNAV messages */
    if (cnav_msg_get(nav_meas[i]->sid, CNAV_MSG_TYPE_30, &cnav_msg) &&
        get_isc_corr(nav_meas[i]->sid.code, &cnav_msg.data.type_30, &isc)) {
      /* remove the already applied TGD correction */
      double tgd = 0.0;
      get_tgd_correction(&ephe[i], &nav_meas[i]->sid, &tgd);
      isc += tgd * GPS_C;
      /* apply the new minus old */
      nav_meas[i]->pseudorange += isc;
      nav_meas[i]->carrier_phase -= isc / sid_to_lambda(nav_meas[i]->sid);
    }
  }
}
