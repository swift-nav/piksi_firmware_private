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

/**
 * Check GPS L1 doppler vs. other tracked GPS L1 satellite doppler
 *
 * This function whitelists L1 satellite pairs with doppler mismatch.
 * GPS L1 satellites with matching doppler are xcorr flagged for investigation.
 * CN0 difference of xcorr flagged satellites is saved.
 *
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be
 * investigated.
 * \param[out]    sat_active      Flags indicating satellites currently tracked.
 * \param[out]    xcorr_cn0_diffs CN0 difference of the satellites to be
 * investigated [dB-Hz].
 * \param[in]     config          Tracker configuration container
 *
 * \return None
 */
static void check_gps_l1_entry(tracker_t *tracker,
                               const tracker_cc_entry_t *entry,
                               bool xcorr_flags[],
                               bool sat_active[],
                               float xcorr_cn0_diffs[],
                               tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  if ((CODE_GPS_L1CA != entry->mesid.code)) {
    /* Ignore other than GPS L1CA for now */
    return;
  }

  if (mesid_is_equal(entry->mesid, tracker->mesid)) {
    /* Ignore self */
    return;
  }

  /* Mark active SVs. Later clear the whitelist status of inactive SVs */
  u16 index = mesid_to_code_index(entry->mesid);
  sat_active[index] = true;
  float cn0 = tracker->cn0;
  float entry_cn0 = entry->cn0;
  float error =
      fabsf(remainder(entry->freq - tracker->xcorr_freq, L1CA_XCORR_FREQ_STEP));

  s32 max_time_cnt = (s32)(2.0f * config->xcorr_time * XCORR_UPDATE_RATE);

  if (error <= config->xcorr_delta) {
    /* Signal pairs with matching doppler are xcorr flagged */
    xcorr_flags[index] = true;
    xcorr_cn0_diffs[index] = cn0 - entry_cn0;
    data->xcorr_whitelist_counts[index] = 0;
  } else if (error >= 5.0f * config->xcorr_delta) {
    if (data->xcorr_whitelist_counts[index] < max_time_cnt) {
      ++data->xcorr_whitelist_counts[index];
    } else {
      /* Signal pair with significant doppler difference is whitelisted
       * after max_time_cnt delay */
      data->xcorr_whitelist[index] = true;
    }
  }
  if (0 != (entry->flags & TRACKER_FLAG_SENSITIVITY_MODE)) {
    data->xcorr_whitelist[index] = false;
    data->xcorr_whitelist_counts[index] = 0;
  }
}

/**
 * Check xcorr flagged GPS L1 satellites.
 *
 * This function increments the counter when GPS L1 dopplers match.
 * If counter reaches maximum value, then based on cn0 differences
 * xcorr flags are set.
 *
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     idx             Index of the entry.
 * \param[in]     xcorr_flags     Flags indicating satellites to be
 * investigated.
 * \param[in]     xcorr_cn0_diffs CN0 difference of the satellites to be
 * investigated [dB-Hz].
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 * \param[in]     config          Tracker configuration container
 *
 * \return None
 */
static void check_gps_l1_xcorr_flags(tracker_t *tracker,
                                     u16 idx,
                                     bool xcorr_flags[],
                                     float xcorr_cn0_diffs[],
                                     bool *xcorr_suspect,
                                     tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  if (idx + 1 == tracker->mesid.sat) {
    /* Exclude self */
    return;
  }

  s32 max_time_cnt = (s32)(config->xcorr_time * XCORR_UPDATE_RATE);

  /* Increment counters if signal pair has not been whitelisted */
  if (xcorr_flags[idx] && !data->xcorr_whitelist[idx]) {
    if (data->xcorr_counts[idx] < max_time_cnt) {
      ++data->xcorr_counts[idx];
    } else {
      if (xcorr_cn0_diffs[idx] < XCORR_SUSPECT_THRESHOLD &&
          xcorr_cn0_diffs[idx] > XCORR_CONFIRM_THRESHOLD) {
        /* If CN0 difference is small, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (xcorr_cn0_diffs[idx] <= XCORR_CONFIRM_THRESHOLD) {
        /* If CN0 difference is large, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset xcorr lock counter */
    data->xcorr_counts[idx] = 0;
  }
}

/**
 * Check GPS L1 doppler vs. GPS L2 doppler of the same SV.
 *
 * GPS L1 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be
 * investigated.
 * \param[in]     config          Tracker configuration container
 *
 * \return false if entry was not L2CM from same SV, true if it was.
 */
static bool check_gps_l2_entries(tracker_t *tracker,
                                 const tracker_cc_entry_t *entry,
                                 bool *xcorr_flag,
                                 tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  if (CODE_GPS_L2CM != entry->mesid.code ||
      entry->mesid.sat != tracker->mesid.sat) {
    /* Ignore other than L2CM from same SV */
    return false;
  }

  u16 index = mesid_to_code_index(tracker->mesid);
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  /* Convert L2 doppler to L1 */
  float entry_freq = entry->freq * L2_to_L1_freq;
  float error =
      fabsf(remainderf(entry_freq - tracker->xcorr_freq, L1CA_XCORR_FREQ_STEP));

  if (error <= config->xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * config->xcorr_delta) {
    /* Signal pairs with mismatching doppler are xcorr flagged */
    *xcorr_flag = true;
  }

  if (tracker->cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD) {
    /* Whitelist high CN0 signal */
    data->xcorr_whitelist[index] = true;
  } else {
    data->xcorr_whitelist[index] = false;
  }

  if (entry->cn0 >= L2CM_XCORR_WHITELIST_THRESHOLD) {
    /* If L2 signal is tracked with decent CN0 and
     * signals are not xcorr flagged,
     * then whitelist the signal */
    data->xcorr_whitelist_l2 = true;
  } else {
    data->xcorr_whitelist_l2 = false;
  }

  return true;
}

/**
 * Check xcorr flagged GPS L2 satellite.
 *
 * This function increments the counter when GPS L2 doppler is not matching.
 * If counter reaches maximum value, then based on whitelist status
 * xcorr flags are set.
 *
 * \param         tracker         Tracker channel data
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 * \param[in]     config          Tracker configuration container
 *
 * \return None
 */
static void check_gps_l2_xcorr_flag(tracker_t *tracker,
                                    bool xcorr_flag,
                                    bool *xcorr_suspect,
                                    tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  u16 index = mesid_to_code_index(tracker->mesid);
  s32 max_time_cnt = (s32)(config->xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l2 < max_time_cnt) {
      ++data->xcorr_count_l2;
    } else {
      if (data->xcorr_whitelist[index] && data->xcorr_whitelist_l2) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist[index] && !data->xcorr_whitelist_l2) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist[index]) {
        /* Otherwise if L1 signal is not whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l2 = 0;
  }
}

/**
 * Check L2 doppler vs. L1 doppler of the same SV.
 *
 * L2 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flag      Flag indicating satellite to be investigated.
 * \param[in]     config          Tracker configuration container
 *
 * \return false if entry was not L1C/A from same SV, true if it was.
 */
bool check_gps_l1_entries(tracker_t *tracker,
                          const tracker_cc_entry_t *entry,
                          bool *xcorr_flag,
                          tp_tracker_config_t *config) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  if (CODE_GPS_L1CA != entry->mesid.code ||
      entry->mesid.sat != tracker->mesid.sat) {
    /* Ignore other than L1C/A from same SV */
    return false;
  }

  /* Convert L2 doppler to L1 */
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  float freq_mod =
      fmodf(tracker->xcorr_freq * L2_to_L1_freq, L1CA_XCORR_FREQ_STEP);

  float entry_cn0 = entry->cn0;
  float entry_freq = entry->freq;
  float entry_freq_mod = fmodf(entry_freq, L1CA_XCORR_FREQ_STEP);
  float error = fabsf(entry_freq_mod - freq_mod);

  if (error <= config->xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * config->xcorr_delta) {
    /* Signal pairs with mismatching doppler are xcorr flagged */
    *xcorr_flag = true;
  }

  if (entry_cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD) {
    data->xcorr_whitelist_l1 = true;
  } else {
    data->xcorr_whitelist_l1 = false;
  }

  if (tracker->cn0 >= L2CM_XCORR_WHITELIST_THRESHOLD) {
    /* If L2 signal is tracked with decent CN0 and
     * signals are not xcorr flagged,
     * then whitelist the signal */
    data->xcorr_whitelist = true;
  } else {
    data->xcorr_whitelist = false;
  }

  return true;
}

/**
 * Check xcorr flagged L1 satellite.
 *
 * This function increments the counter when L1 doppler is not matching.
 * If counter reaches maximum value, then based on whitelist status
 * xcorr flags are set.
 *
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 * \param[in]     config          Tracker configuration container
 *
 * \return None
 */
static void check_gps_l1_xcorr_flag(tracker_t *tracker,
                                    bool xcorr_flag,
                                    bool *xcorr_suspect,
                                    tp_tracker_config_t *config) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;
  s32 max_time_cnt = (s32)(config->xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l1 < max_time_cnt) {
      ++data->xcorr_count_l1;
    } else {
      if (data->xcorr_whitelist && data->xcorr_whitelist_l1) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist && !data->xcorr_whitelist_l1) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist) {
        /* Otherwise if L2 signal is not whitelisted, mark as confirmed xcorr */
        tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l1 = 0;
  }
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
 * \param         tracker         Tracker channel data
 * \param         config          Tracker configuration container
 *
 * \return None
 */
static void update_gps_l1_xcorr(tracker_t *tracker,
                                tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  if (tracker_get_xcorr_flag(tracker)) {
    /* Cross-correlation is set by external thread */
    tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
    return;
  }

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flags[NUM_SATS_GPS] = {false};
  bool sat_active[NUM_SATS_GPS] = {false};
  float xcorr_cn0_diffs[NUM_SATS_GPS] = {0.0f};

  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    check_gps_l1_entry(
        tracker, entry, xcorr_flags, sat_active, xcorr_cn0_diffs, config);
  }

  /* Clear whitelistings of satellites that are no longer tracked */
  for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
    if (!sat_active[idx]) {
      data->xcorr_whitelist[idx] = false;
      data->xcorr_whitelist_counts[idx] = 0;
      data->xcorr_counts[idx] = 0;
    }
  }

  bool sensitivity_mode =
      (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));
  /* If signal is in sensitivity mode, all whitelistings are cleared */
  if (sensitivity_mode) {
    for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
      data->xcorr_whitelist[idx] = false;
      data->xcorr_whitelist_counts[idx] = 0;
    }
  }

  bool xcorr_suspect = false;
  /* Increment counters or Make decision for all xcorr flagged signals */
  for (u16 idx = 0; idx < ARRAY_SIZE(xcorr_flags); ++idx) {
    check_gps_l1_xcorr_flags(
        tracker, idx, xcorr_flags, xcorr_cn0_diffs, &xcorr_suspect, config);
  }

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker);

  tracker_set_xcorr_suspect_flag(
      tracker, xcorr_suspect | prn_check_fail, sensitivity_mode);
}

/**
 * Updates GPS L1 cross-correlation state from GPS L2.
 *
 * This function checks if GPS L1 and GPS L2 have a `mismatching` frequency for
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
 * \param         tracker         Tracker channel data
 * \param         config          Tracker configuration container
 *
 * \return None
 */
static void update_gps_l1_xcorr_from_gps_l2(tracker_t *tracker,
                                            tp_tracker_config_t *config) {
  gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (check_gps_l2_entries(tracker, entry, &xcorr_flag, config)) {
      break;
    }
  }

  u16 index = mesid_to_code_index(tracker->mesid);
  bool sensitivity_mode =
      (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist[index] = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L2 is xcorr flagged */
  check_gps_l2_xcorr_flag(tracker, xcorr_flag, &xcorr_suspect, config);

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker);

  tracker_set_xcorr_suspect_flag(
      tracker, xcorr_suspect | prn_check_fail, sensitivity_mode);
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
 * \param[in,out] tracker         Tracker channel data
 * \param[in]     config          Tracker configuration container
 *
 * \return None
 */
static void update_gps_l2_xcorr_from_gps_l1(tracker_t *tracker,
                                            tp_tracker_config_t *config) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  if (tracker_get_xcorr_flag(tracker)) {
    /* Cross-correlation is set by external thread */
    tracker->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
    return;
  }

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (check_gps_l1_entries(tracker, entry, &xcorr_flag, config)) {
      break;
    }
  }

  bool sensitivity_mode =
      (0 != (tracker->flags & TRACKER_FLAG_SENSITIVITY_MODE));
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L1 is xcorr flagged */
  check_gps_l1_xcorr_flag(tracker, xcorr_flag, &xcorr_suspect, config);

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker);

  tracker_set_xcorr_suspect_flag(
      tracker, xcorr_suspect | prn_check_fail, sensitivity_mode);
}

static bool do_xcorr(tracker_t *tracker) {
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    gps_l1ca_tracker_data_t *data = &tracker->gps_l1ca;
    if (data->xcorr_comp_count++ % 50 == 0) {
      return true;
    }
  } else if (CODE_GPS_L2CM == tracker->mesid.code) {
    gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;
    if (data->xcorr_comp_count++ % 50 == 0) {
      return true;
    }
  } else if (CODE_QZS_L1CA == tracker->mesid.code) {
    /* TODO QZSS: XCORR */
    assert(!"QZSS XCORR implementation missing!");
  }
  return false;
}

void tracker_xcorr_update(tracker_t *tracker, tp_tracker_config_t *config) {
  if (!do_xcorr(tracker)) {
    return;
  }
  if (CODE_GPS_L1CA == tracker->mesid.code) {
    update_gps_l1_xcorr(tracker, config);
    update_gps_l1_xcorr_from_gps_l2(tracker, config);
  } else if (CODE_GPS_L2CM == tracker->mesid.code) {
    update_gps_l2_xcorr_from_gps_l1(tracker, config);
  } else if (CODE_QZS_L1CA == tracker->mesid.code) {
    /* TODO QZSS: XCORR */
    assert(!"QZSS XCORR implementation missing!");
  }
  return;
}
