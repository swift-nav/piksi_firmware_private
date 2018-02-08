/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "xcorr.h"

#include <track/track_flags.h>
#include <track/track_utils.h>

/** GPS/QZSS L1 C/A cross-correlation frequency step [hz] */
#define L1CA_XCORR_FREQ_STEP_HZ (1000.f)
/** GPS/QZSS L1 C/A CN0 threshold for whitelisting [dB-Hz] */
#define L1CA_XCORR_WL_CN0_TH_DBHZ (40.f)
/** GPS/QZSS L2 CM CN0 threshold for whitelisting [dB-Hz] */
#define L2CM_XCORR_WL_CN0_TH_DBHZ (27.f)
/** GPS/QZSS time threshold for whitelisting [s] */
#define XCORR_WL_TIME_TH_S (10)
/** GPS/QZSS frequency error threshold for whitelisting [Hz] */
#define XCORR_WL_FREQ_ERR_TH_HZ (50)
/** GPS/QZSS L1 C/A CN0 threshold for suspected xcorr [dB-Hz] */
#define XCORR_SUSPECT_CN0_TH_DBHZ (-15.f)
/** GPS/QZSS L1 C/A CN0 threshold for confirmed xcorr [dB-Hz] */
#define XCORR_CONFIRM_CN0_TH_DBHZ (-20.f)
/** GPS/QZSS time threshold for xcorr decision [s] */
#define XCORR_TIME_TH_S (5)
/** GPS/QZSS frequency error threshold for xcorr decision [s] */
#define XCORR_FREQ_ERR_TH_HZ (10)
/** cross-correlation update rate divisor */
#define XCORR_UPDATE_RATE_DIVISOR (10)
/** cross-correlation update rate [Hz] */
#define XCORR_UPDATE_RATE_HZ \
  (SECS_MS / GPS_L1CA_BIT_LENGTH_MS / XCORR_UPDATE_RATE_DIVISOR)
/** GPS/QZSS count threshold for whitelisting */
#define XCORR_WL_COUNT_TH (XCORR_WL_TIME_TH_S * XCORR_UPDATE_RATE_HZ)
/** GPS/QZSS count threshold for xcorr decision */
#define XCORR_COUNT_TH (XCORR_TIME_TH_S * XCORR_UPDATE_RATE_HZ)

static void check_l1_sensitivity_mode(tracker_t *tracker) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  bool sensitivity_mode =
      (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));

  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, all whitelistings are cleared */
    for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
      data->xcorr_whitelist[idx] = false;
      data->xcorr_whitelist_counts[idx] = 0;
    }
  }
}

static void check_l2_sensitivity_mode(tracker_t *tracker) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  bool sensitivity_mode =
      (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));

  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist = false;
  }
}

static void check_sensitivity_mode(tracker_t *tracker) {
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    check_l1_sensitivity_mode(tracker);
  } else if (CODE_GPS_L2CM == tracker->mesid.code) {
    check_l2_sensitivity_mode(tracker);
  }
}

static float get_freq_error(const tracker_t *tracker,
                            const tracker_cc_entry_t *entry) {
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  float entry_freq_hz = entry->freq_hz;
  float track_freq_hz = tracker->xcorr_freq_hz;

  if (CODE_GPS_L2CM == entry->mesid.code ||
      CODE_QZS_L2CM == entry->mesid.code) {
    entry_freq_hz *= L2_to_L1_freq;
  } else if (CODE_GPS_L2CM == tracker->mesid.code ||
             CODE_QZS_L2CM == tracker->mesid.code) {
    track_freq_hz *= L2_to_L1_freq;
  }

  float error =
      fabsf(remainderf(entry_freq_hz - track_freq_hz, L1CA_XCORR_FREQ_STEP_HZ));

  return error;
}

/**
 * Check GPS L1 doppler vs. all other tracked GPS L1 satellite dopplers
 *
 * This function whitelists L1 satellite pairs with doppler mismatch.
 * GPS L1 satellites with matching doppler are xcorr flagged for investigation.
 * CN0 difference of xcorr flagged satellites is saved.
 *
 * \param[in,out] tracker     Tracker channel data
 * \param[out]    xcorr_flags Flags indicating satellites to be investigated.
 * \param[out]    sat_active  Flags indicating satellites currently tracked.
 * \param[out]    cn0_diffs   CN0 difference of the satellites to be
 *                            investigated [dB-Hz].
 *
 * \return None
 */
static void check_all_l1_entries(tracker_t *tracker,
                                 bool xcorr_flags[],
                                 bool sat_active[],
                                 float cn0_diffs[]) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);
  u16 ind;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (CODE_GPS_L1CA != entry->mesid.code) {
      /* Ignore other than GPS L1CA for now */
      continue;
    }

    if (mesid_is_equal(entry->mesid, tracker->mesid)) {
      ind = mesid_to_code_index(entry->mesid);
      sat_active[ind] = true;
      /* Ignore self */
      continue;
    }

    /* Mark active SVs. Later clear the whitelist status of inactive SVs */
    ind = mesid_to_code_index(entry->mesid);
    sat_active[ind] = true;

    /* Investigate Doppler differences */
    float error = get_freq_error(tracker, entry);
    if (error <= XCORR_FREQ_ERR_TH_HZ) {
      /* Signal pairs with matching doppler are xcorr flagged */
      xcorr_flags[ind] = true;
      cn0_diffs[ind] = tracker->cn0 - entry->cn0;
      data->xcorr_whitelist_counts[ind] = 0;
    } else if (error >= XCORR_WL_FREQ_ERR_TH_HZ) {
      if (data->xcorr_whitelist_counts[ind] < XCORR_WL_COUNT_TH) {
        ++data->xcorr_whitelist_counts[ind];
      } else {
        /* Signal pair with significant doppler difference is whitelisted
         * after XCORR_WL_TIME_TH_S delay */
        data->xcorr_whitelist[ind] = true;
      }
    }
    if (0 != (entry->flags & TRACKER_FLAG_SENSITIVITY_MODE)) {
      data->xcorr_whitelist[ind] = false;
      data->xcorr_whitelist_counts[ind] = 0;
    }
  }
}

/**
 * Check xcorr flagged GPS L1 satellites.
 *
 * This function increments the counter when GPS L1 dopplers match.
 * If counter reaches maximum value, then based on cn0 differences
 * xcorr flags are set.
 *
 * \param[in,out] tracker     Tracker channel data
 * \param[in]     xcorr_flags Flags indicating satellites to be investigated.
 * \param[in]     cn0_diffs   CN0 difference of the satellites to be
 *                            investigated [dB-Hz].
 *
 * \return None
 */
static bool check_gps_l1_xcorr_flags(tracker_t *tracker,
                                     bool xcorr_flags[],
                                     float cn0_diffs[]) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  bool xcorr_suspect = false;

  for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
    if (idx + 1 == tracker->mesid.sat) {
      /* Exclude self */
      continue;
    }

    /* Increment counters if signal pair has not been whitelisted */
    if (xcorr_flags[idx] && !data->xcorr_whitelist[idx]) {
      if (data->xcorr_counts[idx] < XCORR_COUNT_TH) {
        ++data->xcorr_counts[idx];
      } else {
        if (cn0_diffs[idx] < XCORR_SUSPECT_CN0_TH_DBHZ &&
            cn0_diffs[idx] > XCORR_CONFIRM_CN0_TH_DBHZ) {
          /* If CN0 difference is small, mark as xcorr suspect */
          xcorr_suspect = true;
        } else if (cn0_diffs[idx] <= XCORR_CONFIRM_CN0_TH_DBHZ) {
          /* If CN0 difference is large, mark as confirmed xcorr */
          tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
        }
      }
    } else {
      /* Reset xcorr lock counter */
      data->xcorr_counts[idx] = 0;
    }
  }

  return xcorr_suspect;
}

/**
 * Check GPS L1 vs. GPS L2 of the same SV.
 * GPS L1 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in,out] tracker     Tracker channel data
 *
 * \return true if L2CM was found, and should be investigated.
 */
static bool check_own_l2_entry(tracker_t *tracker) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (CODE_GPS_L2CM != entry->mesid.code ||
        entry->mesid.sat != tracker->mesid.sat) {
      /* Ignore other than L2CM from same SV */
      continue;
    }

    float error = get_freq_error(tracker, entry);
    if (error >= XCORR_WL_FREQ_ERR_TH_HZ) {
      /* L1 & L2 signal pairs with mismatching doppler are xcorr flagged */
      xcorr_flag = true;
    }

    u16 index = mesid_to_code_index(tracker->mesid);

    if (tracker->cn0 >= L1CA_XCORR_WL_CN0_TH_DBHZ) {
      /* Whitelist L1 signal with high CN0  */
      data->xcorr_whitelist[index] = true;
    } else {
      data->xcorr_whitelist[index] = false;
    }

    if (entry->cn0 >= L2CM_XCORR_WL_CN0_TH_DBHZ) {
      /* Whitelist L2 signal with decent CN0 */
      data->xcorr_whitelist_l2 = true;
    } else {
      data->xcorr_whitelist_l2 = false;
    }

    return xcorr_flag;
  }
  return xcorr_flag;
}

/**
 * Check xcorr flagged GPS L2 satellite.
 *
 * This function increments the counter when GPS L2 doppler is not matching.
 * If counter reaches maximum value, then based on whitelist status
 * xcorr flags are set.
 *
 * \param[in,out] tracker       Tracker channel data
 * \param[in]     xcorr_flag    Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect Flag set if satellite is xcorr suspect.
 *
 * \return true if L2CA was found, and should be investigated.
 */
static bool check_gps_l2_xcorr_flag(tracker_t *tracker, bool xcorr_flag) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  bool xcorr_suspect = false;
  u16 index = mesid_to_code_index(tracker->mesid);

  if (xcorr_flag) {
    if (data->xcorr_count_l2 < XCORR_COUNT_TH) {
      ++data->xcorr_count_l2;
    } else {
      if (data->xcorr_whitelist[index] && data->xcorr_whitelist_l2) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist[index] && !data->xcorr_whitelist_l2) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        xcorr_suspect = true;
      } else if (!data->xcorr_whitelist[index]) {
        /* Otherwise if L1 signal is not whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l2 = 0;
  }
  return xcorr_suspect;
}

/**
 * Check GPS L2 vs. GPS L1 of the same SV.
 * GPS L2 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in,out] tracker Tracker channel data
 *
 * \return true if L1CA was found, and should be investigated.
 */
static bool check_own_l1_entry(tracker_t *tracker) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (CODE_GPS_L1CA != entry->mesid.code ||
        entry->mesid.sat != tracker->mesid.sat) {
      /* Ignore other than L1CA from same SV */
      continue;
    }

    float error = get_freq_error(tracker, entry);
    if (error >= XCORR_WL_FREQ_ERR_TH_HZ) {
      /* Signal pairs with mismatching doppler are xcorr flagged */
      xcorr_flag = true;
    }

    float entry_cn0 = entry->cn0;

    if (entry_cn0 >= L1CA_XCORR_WL_CN0_TH_DBHZ) {
      data->xcorr_whitelist_l1 = true;
    } else {
      data->xcorr_whitelist_l1 = false;
    }

    if (tracker->cn0 >= L2CM_XCORR_WL_CN0_TH_DBHZ) {
      data->xcorr_whitelist = true;
    } else {
      data->xcorr_whitelist = false;
    }

    return xcorr_flag;
  }
  return xcorr_flag;
}

/**
 * Check xcorr flagged L1 satellite.
 *
 * This function increments the counter when L1 doppler is not matching.
 * If counter reaches maximum value, then based on whitelist status
 * xcorr flags are set.
 *
 * \param[in,out] tracker       Tracker channel data
 * \param[in]     xcorr_flag    Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect Flag set if satellite is xcorr suspect.
 *
 * \return None
 */
static bool check_gps_l1_xcorr_flag(tracker_t *tracker, bool xcorr_flag) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  bool xcorr_suspect = false;
  if (xcorr_flag) {
    if (data->xcorr_count_l1 < XCORR_COUNT_TH) {
      ++data->xcorr_count_l1;
    } else {
      if (data->xcorr_whitelist && data->xcorr_whitelist_l1) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist && !data->xcorr_whitelist_l1) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        xcorr_suspect = true;
      } else if (!data->xcorr_whitelist) {
        /* Otherwise if L2 signal is not whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l1 = 0;
  }

  return xcorr_suspect;
}

/**
 * Updates GPS L1 cross-correlation state.
 *
 * This function checks if any of the GPS L1 channels have a `matching`
 * frequency
 * for a pre-configured period of time. The match condition is described by:
 *
 * |mod(doppler1,1000) - mod(doppler2,1000)|
 *
 * \f[
 * \left|{\operatorname{Mod}{\left (
 *          \operatorname{LPF}{\left \{doppler_{ch0} \right \} },
 *        1000 \right )} -\
 *        \operatorname{Mod}{\left (
 *          \operatorname{LPF}{\left \{doppler_{ch1} \right \}},
 *        1000 \right )}}\right| < \delta
 * \f]
 *
 * \param[in,out] tracker Tracker channel data
 *
 * \return None
 */
static void update_l1_xcorr(tracker_t *tracker) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  if (tracker_get_xcorr_flag(tracker)) {
    /* Cross-correlation is set by external thread */
    tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
    return;
  }

  bool xcorr_flags[NUM_SATS_GPS] = {false};
  bool sat_active[NUM_SATS_GPS] = {false};
  float cn0_diffs[NUM_SATS_GPS] = {0.0f};

  check_all_l1_entries(tracker, xcorr_flags, sat_active, cn0_diffs);

  /* Clear whitelistings of satellites that are no longer tracked */
  for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
    if (!sat_active[idx]) {
      data->xcorr_whitelist[idx] = false;
      data->xcorr_whitelist_counts[idx] = 0;
      data->xcorr_counts[idx] = 0;
    }
  }

  /* Increment counters or Make decision for all xcorr flagged signals */
  bool xcorr_suspect_l1 =
      check_gps_l1_xcorr_flags(tracker, xcorr_flags, cn0_diffs);

  bool xcorr_flag = check_own_l2_entry(tracker);

  /* Increment counter or Make decision if L2 is xcorr flagged */
  bool xcorr_suspect_l2 = check_gps_l2_xcorr_flag(tracker, xcorr_flag);

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker);

  tracker_set_xcorr_suspect_flag(
      tracker, xcorr_suspect_l1 | xcorr_suspect_l2 | prn_check_fail);
}

/**
 * Updates L2 cross-correlation state from L1.
 *
 * This function checks if L2 and L1 have a `mismatching` frequency for
 * a pre-configured period of time. The mismatch condition is described by:
 *
 * |mod(doppler1,1000) - mod(doppler2,1000)|
 *
 * \f[
 * \left|{\operatorname{Mod}{\left (
 *          \operatorname{LPF}{\left \{doppler_{ch0} \right \} },
 *        1000 \right )} -\
 *        \operatorname{Mod}{\left (
 *          \operatorname{LPF}{\left \{doppler_{ch1} \right \}},
 *        1000 \right )}}\right| < \delta
 * \f]
 *
 * \param[in,out] tracker Tracker channel data
 *
 * \return None
 */
static void update_l2_xcorr(tracker_t *tracker) {
  if (tracker_get_xcorr_flag(tracker)) {
    /* Cross-correlation is set by external thread */
    tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
    return;
  }

  bool xcorr_flag = check_own_l1_entry(tracker);

  /* Increment counter or make decision if L1 is xcorr flagged */
  bool xcorr_suspect = check_gps_l1_xcorr_flag(tracker, xcorr_flag);

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker);

  tracker_set_xcorr_suspect_flag(tracker, xcorr_suspect | prn_check_fail);
}

static bool do_xcorr(tracker_t *tracker) {
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;
    if (data->xcorr_comp_count++ % XCORR_UPDATE_RATE_DIVISOR == 0) {
      return true;
    }
  } else if (CODE_GPS_L2CM == tracker->mesid.code) {
    gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;
    if (data->xcorr_comp_count++ % XCORR_UPDATE_RATE_DIVISOR == 0) {
      return true;
    }
  } else if (CODE_QZS_L1CA == tracker->mesid.code) {
    /* TODO QZSS: XCORR */
    assert(!"QZSS XCORR implementation missing!");
  }
  return false;
}

void tracker_xcorr_update(tracker_t *tracker) {
  if (!do_xcorr(tracker)) {
    return;
  }
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    update_l1_xcorr(tracker);
  } else if (CODE_GPS_L2CM == tracker->mesid.code) {
    update_l2_xcorr(tracker);
  } else if (CODE_QZS_L1CA == tracker->mesid.code) {
    /* TODO QZSS: XCORR */
    assert(!"QZSS XCORR implementation missing!");
  }
  check_sensitivity_mode(tracker);
  return;
}
