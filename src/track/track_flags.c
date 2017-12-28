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

#include "track/track_flags.h"
#include "track.h"
#include "track/track_api.h"

/**
 * The function sets or clears PRN fail flag.
 * Called from Decoder task.
 * \param[in] mesid  ME SV ID
 * \param[in] val prn fail flag value. TRUE if decoded prn from L2C data stream
 *            is not correspond to SVID, otherwise FALSE
 */
void tracker_set_prn_fail_flag(const me_gnss_signal_t mesid, bool val) {
  /* Find SV ID for L1CA and L2CM and set the flag  */
  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; id++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_lock(tracker_channel);
    if (IS_GPS(tracker_channel->mesid) &&
        tracker_channel->mesid.sat == mesid.sat) {
      tracker_channel->prn_check_fail = val;
    }
    tracker_unlock(tracker_channel);
  }
}

/**
 * Sets RAIM exclusion flag to a channel with a given signal identifier
 *
 * \param[in] sid signal identifier for channel to set
 *
 * \return None
 */
void tracker_set_raim_flag(const gnss_signal_t sid) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    /* Find the corresponding channel and flag it. (Note that searching by sid
     * instead of mesid is a bit tricky.. */
    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_lock(tracker_channel);
    /* Is this channel's mesid + orbit slot combination valid? */
    bool can_compare = mesid_valid(tracker_channel->mesid);
    if (IS_GLO(tracker_channel->mesid)) {
      can_compare &= glo_slot_id_is_valid(tracker_channel->glo_orbit_slot);
    }
    if (can_compare && sid_is_equal(mesid2sid(tracker_channel->mesid,
                                              tracker_channel->glo_orbit_slot),
                                    sid)) {
      tracker_channel->flags |= TRACKER_FLAG_RAIM_EXCLUSION;
    }
    tracker_unlock(tracker_channel);
  }
}

/**
 * Sets cross-correlation flag to a channel with a given ME signal identifier
 *
 * \param[in] mesid ME signal identifier for channel to set cross-correlation
 *                  flag.
 *
 * \return None
 */
void tracker_set_xcorr_flag(const me_gnss_signal_t mesid) {
  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    /* Find matching tracker and set the flag  */
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_lock(tracker_channel);
    if (mesid_is_equal(tracker_channel->mesid, mesid)) {
      tracker_channel->xcorr_flag = true;
    }
    tracker_unlock(tracker_channel);
  }
}

/**
 * Sets and clears the L1 & L2 xcorr_suspect flag.
 *
 * This function checks if the xcorr_suspect status has changed for the
 * signal,
 * and sets / clears the flag respectively.
 *
 * \param         tracker_channel Tracker channel data
 * \param[in]     xcorr_suspect     Flag indicating if signal is xcorr
 * suspect.
 * \param[in]     sensitivity_mode  Flag indicating sensitivity mode.
 *
 * \return None
 */
void tracker_set_xcorr_suspect_flag(tracker_channel_t *tracker_channel,
                                    bool xcorr_suspect,
                                    bool sensitivity_mode) {
  if (CODE_GPS_L1CA == tracker_channel->mesid.code) {
    gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  } else {
    gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  }

  if (xcorr_suspect) {
    tracker_channel->flags |= TRACKER_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(tracker_channel->mesid,
                      "setting cross-correlation suspect flag");
    }
  } else {
    tracker_channel->flags &= ~TRACKER_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(tracker_channel->mesid,
                      "clearing cross-correlation suspect flag");
    }
  }
  tracker_channel->xcorr_change_count = tracker_channel->update_count;
}

/**
 * The function checks if PRN fail (decoded prn from L2C data stream
 * is not correspond to SVID) flag set or not.
 * Called from Tracking task.
 * \param[in] tracker_channel Tracker channel data
 * \return    TRUE if PRN fail flag is set, otherwise FAIL
 */
bool tracker_get_prn_fail_flag(tracker_channel_t *tracker_channel) {
  return tracker_channel->prn_check_fail;
}

/**
 * Checks if the tracker has cross-correlation flag set.
 *
 * Tracker can use this method to check if a cross-correlation flag is set by
 * external thread.
 *
 * \param[in] tracker_channel Tracker channel data
 *
 * \return Cross-correlation flag value-
 */
bool tracker_get_xcorr_flag(tracker_channel_t *tracker_channel) {
  return tracker_channel->xcorr_flag;
}
