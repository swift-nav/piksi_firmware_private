/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_gps_l1ca.h"
#include "filters/filter_common.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_gps_l2c.h" /* for L1C/A to L2C tracking handover */

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <libswiftnav/ch_meas.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>

/* STD headers */
#include <string.h>

/** GPS L1 C/A parameter section name */
#define L1CA_TRACK_SETTING_SECTION "l1ca_track"

/** GPS L1 C/A configuration container */
static tp_tracker_config_t gps_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS L1 C/A */
static tracker_interface_function_t tracker_gps_l1ca_init;
static tracker_interface_function_t tracker_gps_l1ca_update;

/** GPS L1 C/A tracker interface */
static const tracker_interface_t tracker_interface_gps_l1ca = {
    .code = CODE_GPS_L1CA,
    .init = tracker_gps_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_gps_l1ca_update,
};

/** Register GPS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l1ca_register(void) {
  lp1_filter_compute_params(&gps_l1ca_config.xcorr_f_params,
                            gps_l1ca_config.xcorr_cof,
                            SECS_MS / GPS_L1CA_BIT_LENGTH_MS);

  tracker_interface_register(&tracker_interface_gps_l1ca);
}

static void tracker_gps_l1ca_init(tracker_t *tracker_channel) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(tracker_channel, &gps_l1ca_config);
}

/**
 * Check L1 doppler vs. other tracked L1 satellite doppler
 *
 * This function whitelists L1 satellite pairs with doppler mismatch.
 * L1 satellites with matching doppler are xcorr flagged for investigation.
 * CN0 difference of xcorr flagged satellites is saved.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be
 * investigated.
 * \param[out]    sat_active      Flags indicating satellites currently tracked.
 * \param[out]    xcorr_cn0_diffs CN0 difference of the satellites to be
 * investigated [dB-Hz].
 *
 * \return None
 */
static void check_L1_entry(tracker_t *tracker_channel,
                           const tracker_cc_entry_t *entry,
                           bool xcorr_flags[],
                           bool sat_active[],
                           float xcorr_cn0_diffs[]) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  if ((CODE_GPS_L1CA != entry->mesid.code) &&
      (CODE_QZS_L1CA != entry->mesid.code)) {
    /* Ignore other than L1CA for now */
    return;
  }

  if (mesid_is_equal(entry->mesid, tracker_channel->mesid)) {
    /* Ignore self */
    return;
  }

  /* Mark active SVs. Later clear the whitelist status of inactive SVs */
  u16 index = mesid_to_code_index(entry->mesid);
  sat_active[index] = true;
  float cn0 = tracker_channel->cn0;
  float entry_cn0 = entry->cn0;
  float error = fabsf(remainder(entry->freq - tracker_channel->xcorr_freq,
                                L1CA_XCORR_FREQ_STEP));

  s32 max_time_cnt =
      (s32)(10.0f * gps_l1ca_config.xcorr_time * XCORR_UPDATE_RATE);

  if (error <= gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with matching doppler are xcorr flagged */
    xcorr_flags[index] = true;
    xcorr_cn0_diffs[index] = cn0 - entry_cn0;
    data->xcorr_whitelist_counts[index] = 0;
  } else if (error >= 5.0f * gps_l1ca_config.xcorr_delta) {
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
 * Check xcorr flagged L1 satellites.
 *
 * This function increments the counter when L1 dopplers match.
 * If counter reaches maximum value, then based on cn0 differences
 * xcorr flags are set.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     idx             Index of the entry.
 * \param[in]     xcorr_flags     Flags indicating satellites to be
 * investigated.
 * \param[in]     xcorr_cn0_diffs CN0 difference of the satellites to be
 * investigated [dB-Hz].
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 *
 * \return None
 */
static void check_L1_xcorr_flags(tracker_t *tracker_channel,
                                 u16 idx,
                                 bool xcorr_flags[],
                                 float xcorr_cn0_diffs[],
                                 bool *xcorr_suspect) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  if (idx + 1 == tracker_channel->mesid.sat) {
    /* Exclude self */
    return;
  }

  s32 max_time_cnt = (s32)(gps_l1ca_config.xcorr_time * XCORR_UPDATE_RATE);

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
        tracker_flag_drop(tracker_channel, CH_DROP_REASON_XCORR);
      }
    }
  } else {
    /* Reset xcorr lock counter */
    data->xcorr_counts[idx] = 0;
  }
}

/**
 * Check L1 doppler vs. L2 doppler of the same SV.
 *
 * L1 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be
 * investigated.
 *
 * \return false if entry was not L2CM from same SV, true if it was.
 */
static bool check_L2_entries(tracker_t *tracker_channel,
                             const tracker_cc_entry_t *entry,
                             bool *xcorr_flag) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  if (CODE_GPS_L2CM != entry->mesid.code ||
      entry->mesid.sat != tracker_channel->mesid.sat) {
    /* Ignore other than L2CM from same SV */
    return false;
  }

  u16 index = mesid_to_code_index(tracker_channel->mesid);
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  /* Convert L2 doppler to L1 */
  float entry_freq = entry->freq * L2_to_L1_freq;
  float error = fabsf(remainderf(entry_freq - tracker_channel->xcorr_freq,
                                 L1CA_XCORR_FREQ_STEP));

  if (error <= gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with mismatching doppler are xcorr flagged */
    *xcorr_flag = true;
  }

  if (tracker_channel->cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD) {
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
 * Check xcorr flagged L2 satellite.
 *
 * This function increments the counter when L2 doppler is not matching.
 * If counter reaches maximum value, then based on whitelist status
 * xcorr flags are set.
 *
 * \param         tracker_channel Tracker channel data
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 *
 * \return None
 */
static void check_L2_xcorr_flag(tracker_t *tracker_channel,
                                bool xcorr_flag,
                                bool *xcorr_suspect) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  u16 index = mesid_to_code_index(tracker_channel->mesid);
  s32 max_time_cnt = (s32)(gps_l1ca_config.xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l2 < max_time_cnt) {
      ++data->xcorr_count_l2;
    } else {
      if (data->xcorr_whitelist[index] && data->xcorr_whitelist_l2) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker_flag_drop(tracker_channel, CH_DROP_REASON_XCORR);
      } else if (!data->xcorr_whitelist[index] && !data->xcorr_whitelist_l2) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist[index]) {
        /* Otherwise if L1 signal is not whitelisted, mark as confirmed xcorr */
        tracker_flag_drop(tracker_channel, CH_DROP_REASON_XCORR);
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l2 = 0;
  }
}

/**
 * Updates L1 cross-correlation state.
 *
 * This function checks if any of the L1 channels have a `matching` frequency
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
 * \param         tracker_channel Tracker channel data
 *
 * \return None
 */
static void update_l1_xcorr(tracker_t *tracker_channel) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  if (tracker_get_xcorr_flag(tracker_channel)) {
    /* Cross-correlation is set by external thread */
    tracker_flag_drop(tracker_channel, CH_DROP_REASON_XCORR);
    return;
  }

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flags[NUM_SATS_GPS] = {false};
  bool sat_active[NUM_SATS_GPS] = {false};
  float xcorr_cn0_diffs[NUM_SATS_GPS] = {0.0f};

  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    check_L1_entry(
        tracker_channel, entry, xcorr_flags, sat_active, xcorr_cn0_diffs);
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
      (0 != (tracker_channel->flags & TRACKER_FLAG_SENSITIVITY_MODE));
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
    check_L1_xcorr_flags(
        tracker_channel, idx, xcorr_flags, xcorr_cn0_diffs, &xcorr_suspect);
  }

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker_channel);

  tracker_set_xcorr_suspect_flag(
      tracker_channel, xcorr_suspect | prn_check_fail, sensitivity_mode);
}

/**
 * Updates L1 cross-correlation state from L2.
 *
 * This function checks if L1 and L2 have a `mismatching` frequency for
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
 * \param         tracker_channel Tracker channel data
 *
 * \return None
 */
static void update_l1_xcorr_from_l2(tracker_t *tracker_channel) {
  gps_l1ca_tracker_data_t *data = &tracker_channel->gps_l1ca;

  tracker_cc_data_t cc_data;
  u16 cnt = tracker_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracker_cc_entry_t *const entry = &cc_data.entries[idx];

    if (check_L2_entries(tracker_channel, entry, &xcorr_flag)) {
      break;
    }
  }

  u16 index = mesid_to_code_index(tracker_channel->mesid);
  bool sensitivity_mode =
      (0 != (tracker_channel->flags & TRACKER_FLAG_SENSITIVITY_MODE));
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist[index] = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L2 is xcorr flagged */
  check_L2_xcorr_flag(tracker_channel, xcorr_flag, &xcorr_suspect);

  bool prn_check_fail = tracker_get_prn_fail_flag(tracker_channel);

  tracker_set_xcorr_suspect_flag(
      tracker_channel, xcorr_suspect | prn_check_fail, sensitivity_mode);
}

static void tracker_gps_l1ca_update(tracker_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &gps_l1ca_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker_channel));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker_channel);

  /* GPS L1 C/A-specific cross-correlation operations */
  update_l1_xcorr(tracker_channel);

  /* GPS L1 C/A-specific L2C cross-correlation operations */
  update_l1_xcorr_from_l2(tracker_channel);

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) &&
                 (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)));
  bool tow_valid = (TOW_UNKNOWN != (tracker_channel->TOW_ms));
  double cn0_threshold_dbhz = TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ;
  cn0_threshold_dbhz += TRACK_CN0_HYSTERESIS_THRES_DBHZ;
  bool cn0_high = (tracker_channel->cn0 > cn0_threshold_dbhz);

  if (inlock && confirmed && tow_valid && cn0_high) {
    /* Start L2C tracker if not running */
    do_l1ca_to_l2c_handover(tracker_channel->sample_count,
                            tracker_channel->mesid.sat,
                            tracker_channel->code_phase_prompt,
                            tracker_channel->carrier_freq,
                            tracker_channel->cn0,
                            tracker_channel->TOW_ms);
  }
}
