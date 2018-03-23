/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "signal_db/signal_db.h"
#include "track_flags.h"
#include "tracker.h"
#include "utils/me_constants.h"

/* Doppler sanity checks. */
static void flag_outliers(tracker_t *tracker) {
  struct track_history *history = &tracker->track_history;
  u32 elapsed_ms = tracker->update_count - history->last_ms;
  float doppler_hz = tracker->carrier_freq;
  if (elapsed_ms >= TRACK_HISTORY_STEP_MS) {
    history->doppler_hz[history->index] = doppler_hz;
    history->cn0_dbhz[history->index] = tracker->cn0;
    history->index++;
    if (ARRAY_SIZE(history->doppler_hz) == history->index) {
      history->valid = true;
      history->index = 0;
    }
    history->last_ms = tracker->update_count;
  }
  if (!history->valid) {
    return;
  }
  if (elapsed_ms < TRACK_HISTORY_STEP_MS) {
    return;
  }

  float old_hz = history->doppler_hz[history->index];
  float diff_hz = fabs(doppler_hz - old_hz);
  if (diff_hz > VEL_DOPPLER_CHANGE_MAX_HZ) {
    log_info_mesid(tracker->mesid,
                   "Velocity outlier: %f Hz (max %f Hz), "
                   "cn0 %f dB-Hz, %" PRIu32 " ms",
                   diff_hz,
                   VEL_DOPPLER_CHANGE_MAX_HZ,
                   tracker->cn0,
                   (u32)TRACK_HISTORY_STEP_MS * TRACK_HISTORY_ARRAY_SIZE);
    tracker->flags |= TRACKER_FLAG_OUTLIER;
    return;
  }

  u8 old_index = ACC_OLD_INDEX(history->index);
  old_hz = history->doppler_hz[old_index];
  diff_hz = fabs(doppler_hz - old_hz);

  if (diff_hz > ACC_DOPPLER_CHANGE_MAX_HZ) {
    log_info_mesid(tracker->mesid,
                   "Acceleration outlier: %f Hz (max %f Hz), "
                   "cn0 %f, %" PRIu32 " ms",
                   diff_hz,
                   ACC_DOPPLER_CHANGE_MAX_HZ,
                   tracker->cn0,
                   (u32)(history->index - old_index) * TRACK_HISTORY_STEP_MS);
    tracker->flags |= TRACKER_FLAG_OUTLIER;
    return;
  }

  old_index = CN0_OLD_INDEX(history->index);
  float old_dbhz = history->cn0_dbhz[old_index];
  bool cn0_drop = (old_dbhz >= CN0_OUTLIER_THRESHOLD_DBHZ) &&
                  (tracker->cn0 < CN0_OUTLIER_THRESHOLD_DBHZ);
  cn0_drop |= (old_dbhz - tracker->cn0) > CN0_OUTLIER_DIFF_DBHZ;
  old_hz = history->doppler_hz[old_index];
  diff_hz = fabs(doppler_hz - old_hz);

  if (cn0_drop && (diff_hz > CN0_DROP_DOPPLER_DIFF_HZ)) {
    log_info_mesid(tracker->mesid,
                   "Low CN0 high acceleration outlier:"
                   "%f Hz (max %f Hz), "
                   "cn0 %f, %" PRIu32 " ms",
                   diff_hz,
                   CN0_DROP_DOPPLER_DIFF_HZ,
                   tracker->cn0,
                   (u32)(history->index - old_index) * TRACK_HISTORY_STEP_MS);
    tracker->flags |= TRACKER_FLAG_OUTLIER;
  }
}

/**
 * Drops channels with measurement outliers.
 *
 * Check if an unexpected measurement is done and if so, flags the
 * channel for disposal
 *
 * \param[in,out] tracker Tracker channel data
 */
void tp_tracker_flag_outliers(tracker_t *tracker) {
  const float doppler_max_hz = code_to_sv_doppler_max(tracker->mesid.code) +
                               code_to_tcxo_doppler_max(tracker->mesid.code);

  if (fabsf(tracker->carrier_freq) > doppler_max_hz) {
    log_debug_mesid(
        tracker->mesid, "Doppler %.2f too high", tracker->carrier_freq);
    (tracker->flags) |= TRACKER_FLAG_OUTLIER;
  }

  if (0 == (tracker->flags & TRACKER_FLAG_CONFIRMED)) {
    return;
  }
  if (IS_SBAS(tracker->mesid)) { /* not used for ranging */
    return;
  }
  flag_outliers(tracker);
}
