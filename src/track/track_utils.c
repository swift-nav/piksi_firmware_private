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
#include "track_api.h"
#include "track_flags.h"
#include "track_state.h"
#include "track_utils.h"

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
void tracker_measurement_get(u64 ref_tc,
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
  meas->lock_time = tracker_get_lock_time(time_info, misc_info);
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
bool tracker_calc_pseudorange(u64 ref_tc,
                              const channel_measurement_t *meas,
                              double *raw_pseudorange) {
  navigation_measurement_t nav_meas, *p_nav_meas = &nav_meas;
  gps_time_t rec_time = napcount2gpstime(ref_tc);
  if (!gps_time_valid(&rec_time)) {
    log_warn("Invalid gps time in tracker_calc_pseudorange");
    return false;
  }

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

/**
 * Computes the lock time from tracking channel time info.
 *
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 *
 * \return Lock time [s]
 */
double tracker_get_lock_time(const tracker_time_info_t *time_info,
                             const tracker_misc_info_t *misc_info) {
  u64 lock_time_ms = UINT64_MAX;

  if (0 != misc_info->carrier_phase_offset.value) {
    u64 now_ms = timing_getms();
    assert(now_ms >= misc_info->carrier_phase_offset.timestamp_ms);
    lock_time_ms = now_ms - misc_info->carrier_phase_offset.timestamp_ms;
  }
  lock_time_ms = MIN(lock_time_ms, time_info->ld_pess_locked_ms);

  return (double)lock_time_ms / SECS_MS;
}

/**
 * Loads data relevant to cross-correlation processing
 *
 * The method loads information from all trackers for cross-correlation
 * algorithm.
 *
 * \param[out] cc_data Destination container
 *
 * \return Number of entries loaded
 *
 * \sa tracker_cc_data_t
 */
u16 tracker_load_cc_data(tracker_cc_data_t *cc_data) {
  u16 cnt = 0;

  for (u8 id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    tracker_t *tracker = tracker_get(id);
    tracker_cc_entry_t entry;

    entry.id = id;
    entry.mesid = tracker->mesid;
    entry.flags = tracker->flags;
    entry.freq = tracker->xcorr_freq;
    entry.cn0 = tracker->cn0;

    if (0 != (entry.flags & TRACKER_FLAG_ACTIVE) &&
        0 != (entry.flags & TRACKER_FLAG_CONFIRMED) &&
        0 != (entry.flags & TRACKER_FLAG_XCORR_FILTER_ACTIVE)) {
      cc_data->entries[cnt++] = entry;
    }
  }

  return cnt;
}

/**
 * Atomically updates carrier phase offset.
 *
 * The method locates tracking channel object, locks it, and updates the
 * carrier phase offset only if the channel is still active, belongs to the
 * same signal and has the same lock counter.
 *
 * \param[in] info                 Generic tracking channel information block
 *                                 used for locating destination channel and
 *                                 checking integrity.
 * \param[in] carrier_phase_offset Carrier phase offset to set.
 *
 * \return None
 */
void tracker_set_carrier_phase_offset(const tracker_info_t *info,
                                      s64 carrier_phase_offset) {
  bool adjusted = false;
  tracker_t *tracker = tracker_get(info->id);

  tracker_lock(tracker);
  if (0 != (tracker->flags & TRACKER_FLAG_ACTIVE) &&
      mesid_is_equal(info->mesid, tracker->mesid) &&
      info->lock_counter == tracker->lock_counter) {
    tracker_misc_info_t *misc_info = &tracker->misc_info;
    misc_info->carrier_phase_offset.value = carrier_phase_offset;
    misc_info->carrier_phase_offset.timestamp_ms = timing_getms();
    adjusted = true;
  }
  tracker_unlock(tracker);

  if (adjusted) {
    log_debug_mesid(info->mesid,
                    "Adjusting carrier phase offset to %" PRId64,
                    carrier_phase_offset);
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
    tracker_t *tracker = tracker_get(i);
    if (mesid_is_equal(tracker->mesid, mesid)) {
      return tracker;
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
void tracker_drop_unhealthy_glo(const me_gnss_signal_t mesid) {
  assert(IS_GLO(mesid));
  tracker_t *tracker = tracker_channel_get_by_mesid(mesid);
  if (tracker == NULL) {
    return;
  }
  /* Double-check that channel is in enabled state.
   * Similar check exists in manage_track() in manage.c
   */
  if (!(tracker->busy)) {
    return;
  }
  tracker->flags |= TRACKER_FLAG_GLO_HEALTH_DECODED;
  tracker->health = SV_UNHEALTHY;
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

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param mesid        ME signal ID.
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples) {
  /* Calculate the code phase rate with carrier aiding. */
  double code_phase_rate = (1.0 + carrier_freq / mesid_to_carr_freq(mesid)) *
                           code_to_chip_rate(mesid.code);
  code_phase += n_samples * code_phase_rate / NAP_FRONTEND_SAMPLE_RATE_Hz;
  u32 cp_int = floor(code_phase);
  code_phase -= cp_int - (cp_int % code_to_chip_count(mesid.code));
  return code_phase;
}
