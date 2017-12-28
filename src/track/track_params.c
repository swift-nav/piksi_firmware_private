/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>

#include <libswiftnav/nav_meas.h>

#include "board/nap/track_channel.h"
#include "calc_nav_meas.h"
#include "timing/timing.h"
#include "track_flags.h"
#include "track_params.h"
#include "track_state.h"

/**
 * Converts tracking channel data blocks into channel measurement structure.
 *
 * The method populates measurement fields according to provided values.
 *
 * \param[in]  ref_tc    Reference timing count.
 * \param[in]  info      Generic tracking channel information block.
 * \param[in]  freq_info Frequency and phase information block.
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 * \param[out] meas      Pointer to output channel_measurement_t.
 *
 * \return None
 */
void tracking_channel_measurement_get(u64 ref_tc,
                                      const tracker_info_t *info,
                                      const tracker_freq_info_t *freq_info,
                                      const tracker_time_info_t *time_info,
                                      const tracker_misc_info_t *misc_info,
                                      channel_measurement_t *meas) {
  /* Update our channel measurement. */
  memset(meas, 0, sizeof(*meas));

  meas->sid = mesid2sid(info->mesid, info->glo_orbit_slot);
  meas->code_phase_chips = freq_info->code_phase_chips;
  meas->code_phase_rate = freq_info->code_phase_rate;
  meas->carrier_phase = freq_info->carrier_phase;
  meas->carrier_freq = freq_info->carrier_freq;
  meas->time_of_week_ms = info->tow_ms;
  meas->tow_residual_ns = info->tow_residual_ns;

  meas->rec_time_delta = (double)((s32)(info->sample_count - (u32)ref_tc)) /
                         NAP_FRONTEND_SAMPLE_RATE_Hz;

  meas->cn0 = info->cn0;
  meas->lock_time = tracking_channel_get_lock_time(time_info, misc_info);
  meas->time_in_track = time_info->cn0_usable_ms / 1000.0;
  meas->elevation = TRACKING_ELEVATION_UNKNOWN;
  meas->flags = 0;
}

/**
 * Computes raw pseudorange in [m]
 *
 * \param[in]  ref_tc Reference time
 * \param[in]  meas   Pre-populated channel measurement
 * \param[out] raw_pseudorange Computed pseudorange [m]
 *
 * \retval true Pseudorange is valid
 * \retval false Error in computation.
 */
bool tracking_channel_calc_pseudorange(u64 ref_tc,
                                       const channel_measurement_t *meas,
                                       double *raw_pseudorange) {
  navigation_measurement_t nav_meas, *p_nav_meas = &nav_meas;
  gps_time_t rec_time = napcount2gpstime(ref_tc);
  s8 nm_ret = calc_navigation_measurement(1, &meas, &p_nav_meas, &rec_time);
  if (nm_ret != 0) {
    log_warn_sid(meas->sid,
                 "calc_navigation_measurement() returned an error: %" PRId8,
                 nm_ret);
    return false;
  }
  *raw_pseudorange = nav_meas.raw_pseudorange;
  return true;
}

/** Adjust all carrier phase offsets with a receiver clock correction.
 * Note that as this change to carrier is equal to the change caused to
 * pseudoranges by the clock correction, the code-carrier difference does
 * not change and thus we do not reset the lock counter.
 *
 * \param dt      Receiver clock change (s)
 */
void tracking_channel_carrier_phase_offsets_adjust(double dt) {
  /* Carrier phase offsets are adjusted for all signals matching SPP criteria */
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    me_gnss_signal_t mesid;
    double carrier_phase_offset = 0.0;
    bool adjusted = false;

    tracker_t *tracker_channel = tracker_get(i);
    tracker_pub_data_t *pub_data = &tracker_channel->pub_data;
    volatile tracker_misc_info_t *misc_info = &pub_data->misc_info;

    chMtxLock(&tracker_channel->mutex_pub);
    if (0 != (pub_data->gen_info.flags & TRACKER_FLAG_ACTIVE)) {
      carrier_phase_offset = misc_info->carrier_phase_offset.value;

      /* touch only channels that have the initial offset set */
      if (carrier_phase_offset != 0.0) {
        mesid = pub_data->gen_info.mesid;
        carrier_phase_offset -= mesid_to_carr_freq(mesid) * dt;
        misc_info->carrier_phase_offset.value = carrier_phase_offset;
        /* Note that because code-carrier difference does not change here,
         * we do not reset the lock time carrier_phase_offset.timestamp_ms */
        adjusted = true;
      }
    }
    chMtxUnlock(&tracker_channel->mutex_pub);

    if (adjusted) {
      log_debug_mesid(
          mesid, "Adjusting carrier phase offset to %f", carrier_phase_offset);
    }
  }
}

/** Utility function to find tracking channel allocated to the given mesid.
 *
 * \param[in] mesid ME signal identifier.
 *
 * \return tracker channel container for the requested mesid.
 */
tracker_t *tracker_channel_get_by_mesid(const me_gnss_signal_t mesid) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_t *tracker_channel = tracker_get(i);
    if (mesid_is_equal(tracker_channel->mesid, mesid)) {
      return tracker_channel;
    }
  }
  return NULL;
}

/** Drop unhealthy GLO signal.
 *
 *  Both L1CA and L2CA decode the health information independently.
 *  In case one channel does not contain valid data,
 *  it cannot detect unhealthy status.
 *
 *  If one channel is marked unhealthy,
 *  then also drop the other channel.
 *
 *  This function is called from both GLO L1 and L2 trackers.
 *
 * \param[in] mesid ME signal to be dropped.
 *
 * \return None
 */
void tracking_channel_drop_unhealthy_glo(const me_gnss_signal_t mesid) {
  assert(IS_GLO(mesid));
  tracker_t *tracker_channel = tracker_channel_get_by_mesid(mesid);
  if (tracker_channel == NULL) {
    return;
  }
  /* Double-check that channel is in enabled state.
   * Similar check exists in manage_track() in manage.c
   */
  if (STATE_ENABLED != tracker_state_get(tracker_channel)) {
    return;
  }
  tracker_channel->flags |= TRACKER_FLAG_GLO_HEALTH_DECODED;
  tracker_channel->health = GLO_SV_UNHEALTHY;
}

/**
 * Check validity of handover code phase.
 *
 * The code phase is expected to be near zero at the moment of handover.
 * If the code has rolled over just recently, then it is [0, TOLERANCE]
 * If the code hasn't rolled over yet, then it is [MAX_CHIPS - TOL, MAX_CHIPS)
 * Code phase should never be negative.
 *
 * \param[in] code_phase_chips code phase [chips].
 * \param[in] max_chips        code length of the signal
 *                             from where handover is done [chips].
 *
 * \return true if the code phase is valid, false otherwise.
 */
bool handover_valid(double code_phase_chips, double max_chips) {
  if ((code_phase_chips < 0) ||
      ((code_phase_chips > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase_chips < (max_chips - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    return false;
  }
  return true;
}
