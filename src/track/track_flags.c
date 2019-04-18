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

#include "track_flags.h"

#include "board/nap/track_channel.h"
#include "platform_signal.h"
#include "track_api.h"
#include "track_state.h"

void tracker_flag_drop(tracker_t *tracker, ch_drop_reason_t reason) {
  tracker->flags |= TRACKER_FLAG_DROP_CHANNEL;
  tracker->ch_drop_reason = reason;
}

/**
 * The function sets or clears PRN fail flag.
 * Called from Decoder task.
 * \param[in] mesid  ME SV ID
 * \param[in] val prn fail flag value. TRUE if decoded prn from L2C data stream
 *            is not correspond to SVID, otherwise FALSE
 */
void tracker_set_prn_fail_flag(const me_gnss_signal_t mesid, bool val) {
  /* Find SV ID for L1CA and L2CM and set the flag  */
  for (u8 id = 0; id < nap_track_n_channels; id++) {
    tracker_t *tracker = tracker_get(id);
    tracker_lock(tracker);
    /* Skip inactive channels */
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      tracker_unlock(tracker);
      continue;
    }
    if (IS_GPS(tracker->mesid) && tracker->mesid.sat == mesid.sat) {
      tracker->prn_check_fail = val;
    }
    tracker_unlock(tracker);
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
    tracker_t *tracker = tracker_get(i);
    tracker_lock(tracker);
    /* Skip inactive channels */
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      tracker_unlock(tracker);
      continue;
    }
    /* Is this channel's mesid + orbit slot combination valid? */
    bool can_compare = mesid_valid(tracker->mesid);
    if (IS_GLO(tracker->mesid)) {
      can_compare &= glo_slot_id_is_valid(tracker->glo_orbit_slot);
    }
    if (can_compare &&
        sid_is_equal(mesid2sid(tracker->mesid, tracker->glo_orbit_slot), sid)) {
      tracker_flag_drop(tracker, CH_DROP_REASON_RAIM);
    }
    tracker_unlock(tracker);
  }
}

/**
 * Initiates SBAS tracker drop procedure
 */
void tracker_set_sbas_provider_change_flag(void) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_t *tracker = tracker_get(i);

    tracker_lock(tracker);
    /* Skip inactive channels */
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      tracker_unlock(tracker);
      continue;
    }

    bool sbas_found = IS_SBAS(tracker->mesid);
    if (sbas_found) {
      tracker_flag_drop(tracker, CH_DROP_REASON_SBAS_PROVIDER_CHANGE);
    }

    tracker_unlock(tracker);

    if (sbas_found) {
      break; /* by design, only one SBAS signal is expected in tracker */
    }
  }
}

/**
 * Initiates GLO signals drop procedure due to leap second event
 */
void tracker_set_leap_second_flag(void) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_t *tracker = tracker_get(i);

    tracker_lock(tracker);
    /* Skip inactive channels */
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      tracker_unlock(tracker);
      continue;
    }

    if (IS_GLO(tracker->mesid)) {
      tracker_flag_drop(tracker, CH_DROP_REASON_LEAP_SECOND);
    }

    tracker_unlock(tracker);
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
  for (u8 id = 0; id < nap_track_n_channels; ++id) {
    /* Find matching tracker and set the flag  */
    tracker_t *tracker = tracker_get(id);
    tracker_lock(tracker);
    /* Skip inactive channels */
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      tracker_unlock(tracker);
      continue;
    }
    if (mesid_is_equal(tracker->mesid, mesid)) {
      tracker->xcorr_flag = true;
    }
    tracker_unlock(tracker);
  }
}

/**
 * Sets and clears the L1 & L2 xcorr_suspect flag.
 *
 * This function checks if the xcorr_suspect status has changed for the
 * signal,
 * and sets / clears the flag respectively.
 *
 * \param         tracker Tracker channel data
 * \param[in]     xcorr_suspect     Flag indicating if signal is xcorr
 * suspect.
 * \param[in]     sensitivity_mode  Flag indicating sensitivity mode.
 *
 * \return None
 */
void tracker_set_xcorr_suspect_flag(tracker_t *tracker,
                                    bool xcorr_suspect,
                                    bool sensitivity_mode) {
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  } else {
    gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;
    if ((data->xcorr_flag) == xcorr_suspect) {
      return;
    }
    data->xcorr_flag = xcorr_suspect;
  }

  if (xcorr_suspect) {
    tracker->flags |= TRACKER_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(tracker->mesid, "setting cross-correlation suspect flag");
    }
  } else {
    tracker->flags &= ~TRACKER_FLAG_XCORR_SUSPECT;
    if (!sensitivity_mode) {
      log_debug_mesid(tracker->mesid,
                      "clearing cross-correlation suspect flag");
    }
  }
  tracker->xcorr_change_count = tracker->update_count;
}

/**
 * Update #TRACKER_FLAG_BIT_POLARITY_KNOWN and
 * #TRACKER_FLAG_BIT_INVERTED flags based on
 * tracker_t::bit_polarity value.
 *
 * \param[in,out] tracker Tracker data.
 */
void tracker_update_bit_polarity_flags(tracker_t *tracker) {
  if ((BIT_POLARITY_UNKNOWN != tracker->bit_polarity) &&
      (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) {
    /* Nav bit polarity is known, i.e. half-cycles have been resolved.
     * bit polarity known flag is set only when phase lock to prevent the
     * situation when channel loses an SV, but decoder just finished TOW
     * decoding
     * which cause bit polarity know flag set */
    tracker->flags |= TRACKER_FLAG_BIT_POLARITY_KNOWN;
  } else {
    tracker->flags &= ~TRACKER_FLAG_BIT_POLARITY_KNOWN;
  }

  if (tracker->flags & TRACKER_FLAG_BIT_POLARITY_KNOWN) {
    if (BIT_POLARITY_INVERTED == tracker->bit_polarity) {
      tracker->flags |= TRACKER_FLAG_BIT_INVERTED;
    } else {
      tracker->flags &= ~TRACKER_FLAG_BIT_INVERTED;
    }
  }
}

/**
 * The function checks if PRN fail (decoded prn from L2C data stream
 * is not correspond to SVID) flag set or not.
 * Called from Tracking task.
 * \param[in] tracker Tracker channel data
 * \return    TRUE if PRN fail flag is set, otherwise FAIL
 */
bool tracker_get_prn_fail_flag(tracker_t *tracker) {
  return tracker->prn_check_fail;
}

/**
 * Checks if the tracker has cross-correlation flag set.
 *
 * Tracker can use this method to check if a cross-correlation flag is set by
 * external thread.
 *
 * \param[in] tracker Tracker channel data
 *
 * \return Cross-correlation flag value-
 */
bool tracker_get_xcorr_flag(tracker_t *tracker) { return tracker->xcorr_flag; }
