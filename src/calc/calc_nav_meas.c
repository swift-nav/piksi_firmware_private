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

#include <libswiftnav/cycle_slip.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/troposphere.h>

/* Warning threshold for the difference between TDCP and measured Doppler.
 * (85Hz threshold allows for max 1 ms error in the timestamp difference.) */
#define TDCP_MAX_DELTA_HZ 85

/* Convert a single channel measurement into a single navigation measurement. */
static s8 convert_channel_measurement_to_navigation_measurement(
    const gps_time_t *rec_time,
    const channel_measurement_t *meas,
    navigation_measurement_t *nm) {
  nm->sid = meas->sid;

  u32 code_length = code_to_chip_count(meas->sid.code);
  u32 chips_in_millisecond =
      code_length / code_to_prn_period_ms(meas->sid.code);
  double chip_rate = code_to_chip_rate(meas->sid.code);
  double lambda = sid_to_lambda(meas->sid);

  /* Compute the time of transmit of the signal on the satellite from the
   * tracking loop parameters. This will be used to compute the pseudorange.
   */
  nm->tot.wn = WN_UNKNOWN;
  nm->tot.tow = 1e-3 * meas->time_of_week_ms;
  double chips = meas->code_phase_chips;
  if (chips > code_length) {
    /* Sanity check of the code phase measurement */
    log_warn_sid(nm->sid, "Measured code phase exceeds code length");
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
  nm->tot.tow += chips / chip_rate;

  nm->tot.tow += meas->tow_residual_ns * 1e-9;

  normalize_gps_time(&nm->tot);

  /* Match the week number to the time of reception. */
  gps_time_match_weeks(&nm->tot, rec_time);

  /* Compute the carrier phase measurement. */
  nm->raw_carrier_phase = meas->carrier_phase;

  /* For raw Doppler we use the instantaneous carrier frequency from the
   * tracking loop. */
  nm->raw_measured_doppler = meas->carrier_freq;

  /* Get the approximate elevation from track DB */
  if (!track_sid_db_elevation_degrees_get(nm->sid, &nm->elevation)) {
    /* Use 0 degrees as unknown elevation to assign it the smallest weight */
    log_debug_sid(nm->sid, "Elevation unknown, using 0");
    nm->elevation = 0;
  }

  /* Copy over remaining values. */
  nm->cn0 = meas->cn0;
  nm->lock_time = meas->lock_time;

  /* Measurement time offset from rec_time, usually -5ms .. -0ms */
  double dt = meas->rec_time_delta;

  /* Form the time of reception of this signal */
  gps_time_t meas_tor = *rec_time;
  meas_tor.tow += dt;
  normalize_gps_time(&meas_tor);

  /* The raw pseudorange is just the time of flight multiplied by the speed of
   * light. */
  nm->raw_pseudorange = GPS_C * (gpsdifftime(&meas_tor, &nm->tot));

  /* Finally, propagate measurement back to reference time */
  nm->tot.tow -= dt;
  normalize_gps_time(&nm->tot);

  /* Propagate pseudorange with raw doppler times wavelength */
  nm->raw_pseudorange += dt * nm->raw_measured_doppler * lambda;
  /* Propagate carrier phase with carrier frequency */
  nm->raw_carrier_phase += dt * nm->raw_measured_doppler;

  /* Initialize the fields for corrected measurements */
  nm->pseudorange = nm->raw_pseudorange;
  nm->carrier_phase = nm->raw_carrier_phase;
  nm->measured_doppler = nm->raw_measured_doppler;

  /* Compute flags.
   *
   * \note currently algorithm uses 1 to 1 flag mapping, however it can use
   *       channel measurement flags to compute errors and weight factors.
   */
  nm->flags = 0;
  if (0 != (meas->flags & CHAN_MEAS_FLAG_CODE_VALID)) {
    nm->flags |= NAV_MEAS_FLAG_CODE_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_PHASE_VALID)) {
    nm->flags |= NAV_MEAS_FLAG_PHASE_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    nm->flags |= NAV_MEAS_FLAG_MEAS_DOPPLER_VALID;
  }
  if (0 != (meas->flags & CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN)) {
    nm->flags |= NAV_MEAS_FLAG_HALF_CYCLE_KNOWN;
  }
  nm->flags |= NAV_MEAS_FLAG_CN0_VALID;

  nm->IODE = INVALID_IODE;
  nm->IODC = INVALID_IODC;

  return 0;
}

/** Calculate observations from tracking channel measurements.
 *
 * \param n_channels Number of tracking channel measurements
 * \param meas Array of pointers to tracking channel measurements, length
 *             `n_channels`
 * \param nav_meas Array of pointers of where to store the output observations,
 *                 length `n_channels`
 * \param rec_time Pointer to an estimate of the GPS time at reception time
 * \return '0' for success, '-1' for measurement sanity check error
 */
s8 calc_navigation_measurement(u8 n_channels,
                               const channel_measurement_t *meas[],
                               navigation_measurement_t *nav_meas[],
                               const gps_time_t *rec_time) {
  /* To calculate the pseudorange from the time of transmit we need the local
   * time of reception. */
  if (!gps_time_valid(rec_time)) {
    log_error("Invalid gps time in calc_navigation_measurement n_channels = %u",
              n_channels);
    return -1;
  }

  assert(n_channels <= MAX_CHANNELS);
  for (u8 i = 0; i < n_channels; ++i) {
    s8 ret = convert_channel_measurement_to_navigation_measurement(
        rec_time, meas[i], nav_meas[i]);
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
                        const ephemeris_t *p_ephe[]) {
  u8 i = 0;
  for (i = 0; i < n_channels; i++) {
    double isc;
    cnav_msg_t cnav_msg;
    /* get GPS inter-signal corrections from CNAV messages */
    if (cnav_msg_get(nav_meas[i]->sid, CNAV_MSG_TYPE_30, &cnav_msg) &&
        get_isc_corr(nav_meas[i]->sid.code, &cnav_msg.data.type_30, &isc)) {
      /* remove the already applied TGD correction */
      isc += get_tgd_correction(p_ephe[i], &nav_meas[i]->sid) * GPS_C;
      /* apply the new minus old */
      nav_meas[i]->pseudorange += isc;
      nav_meas[i]->carrier_phase -= isc / sid_to_lambda(nav_meas[i]->sid);
    }
  }
}

/** Set measurement precise Doppler using time difference of carrier phase.
 * \note The return array `m_tdcp` should have space to contain the number
 * of measurements with common PRNs between `m_new` and `m_old`. Making the
 * array at least `MIN(n_new, n_old)` long will ensure sufficient space.
 *
 * \param n_new Number of measurements in `m_new`
 * \param m_new Array of new navigation measurements
 * \param n_old Number of measurements in `m_old`
 * \param m_old Array of old navigation measurements, sorted by PRN
 * \param m_corrected Array in which to store the output measurements
 * \param dt The difference in receiver time between the two measurements
 * \return The number of measurements written to `m_tdcp`
 */
u8 tdcp_doppler(u8 n_new,
                navigation_measurement_t *m_new,
                u8 n_old,
                navigation_measurement_t *m_old,
                navigation_measurement_t *m_corrected,
                double dt) {
  /* Sort m_new, m_old should already be sorted. */
  qsort(m_new, n_new, sizeof(navigation_measurement_t), nav_meas_cmp);

  u8 i, j, n = 0;

  /* Loop over m_new and m_old and check if a PRN is present in both. */
  for (i = 0, j = 0; i < n_new;) {
    if (j >= n_old || sid_compare(m_new[i].sid, m_old[j].sid) < 0) {
      /* No matching old measurement, copy m_new to m_corrected. */
      memcpy(&m_corrected[n], &m_new[i], sizeof(navigation_measurement_t));
      m_corrected[n].raw_computed_doppler = 0.0;
      m_corrected[n].computed_doppler = 0.0;
      m_corrected[n].flags &= ~NAV_MEAS_FLAG_COMP_DOPPLER_VALID;
      n++;
      i++;
    } else if (sid_compare(m_new[i].sid, m_old[j].sid) > 0) {
      /* No matching new measurement, skip the old one */
      j++;
    } else {
      /* Old and new match, copy m_new to m_corrected. */
      memcpy(&m_corrected[n], &m_new[i], sizeof(navigation_measurement_t));
      m_corrected[n].raw_computed_doppler = 0.0;
      m_corrected[n].computed_doppler = 0.0;
      m_corrected[n].flags &= ~NAV_MEAS_FLAG_COMP_DOPPLER_VALID;

      /* If there was a cycle slip or invalid carrier phase,
       * don't set calculated doppler */
      if (!was_cycle_slip(m_old[j].lock_time, m_new[i].lock_time, dt) &&
          (0 != (m_new[i].flags & NAV_MEAS_FLAG_PHASE_VALID)) &&
          (0 != (m_old[j].flags & NAV_MEAS_FLAG_PHASE_VALID))) {
        /* Calculate raw Doppler from time difference of carrier phase.
         * NOTE: flip the CP sign here */
        double computed_doppler =
            (m_new[i].raw_carrier_phase - m_old[j].raw_carrier_phase) / dt;

        /* Reconstruct the Doppler correction between raw and corrected.
         * We don't need to check valid flag this is valid as correction is
         * always applied. */
        double dopp_corr =
            m_new[i].measured_doppler - m_new[i].raw_measured_doppler;

        /* Use the computed TDCP for raw Doppler measurement */
        m_corrected[n].raw_computed_doppler = computed_doppler;
        m_corrected[n].flags |= NAV_MEAS_FLAG_COMP_DOPPLER_VALID;
        /* Apply the correction to form corrected TDCP doppler */
        m_corrected[n].computed_doppler = computed_doppler + dopp_corr;
        m_corrected[n].computed_doppler_dt = dt;

        /* Log a warning if TDCP differed radically from measured */
        if ((fabs(computed_doppler - m_new[i].raw_measured_doppler) >
             TDCP_MAX_DELTA_HZ) &&
            (0 != (m_new[i].flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID))) {
          log_warn_sid(
              m_new[i].sid,
              "TDCP %.1f differs from measured Doppler %.1f (dt = %.6f)",
              computed_doppler,
              m_new[i].raw_measured_doppler,
              dt);
        }
      }

      n++;
      i++;
      j++;
    }
  }
  /* Number of measurements should not change */
  assert(n == n_new);
  return n;
}
