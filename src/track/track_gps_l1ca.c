/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
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
#include "track_gps_l2cm.h" /* for L1C/A to L2 CM tracking handover */
#include "track_cn0.h"
#include "track_profile_utils.h"
#include "track_profiles.h"
#include "track_sid_db.h"

/* Non-local headers */
#include <platform_track.h>
#include <signal.h>
#include <track_api.h>
#include <manage.h>
#include <track.h>
#include <ndb.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <string.h>


/** GPS L1 C/A parameter section name */
#define L1CA_TRACK_SETTING_SECTION "l1ca_track"


/** GPS L1 C/A configuration container */
static tp_tracker_config_t gps_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L1 C/A tracker table */
static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS];
/** GPS L1 C/A tracker data */
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[ARRAY_SIZE(gps_l1ca_trackers)];

/* Forward declarations of interface methods for GPS L1 C/A */
static tracker_interface_function_t tracker_gps_l1ca_init;
static tracker_interface_function_t tracker_gps_l1ca_disable;
static tracker_interface_function_t tracker_gps_l1ca_update;

/** GPS L1 C/A tracker interface */
static const tracker_interface_t tracker_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         tracker_gps_l1ca_init,
  .disable =      tracker_gps_l1ca_disable,
  .update =       tracker_gps_l1ca_update,
  .trackers =     gps_l1ca_trackers,
  .num_trackers = ARRAY_SIZE(gps_l1ca_trackers)
};

/** GPS L1 C/A tracker interface list element */
static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

/**
 * Function for updating configuration on parameter change
 *
 * \param[in] s   Setting descriptor
 * \param[in] val New parameter value
 *
 * \return Update status
 */
static bool settings_pov_speed_cof_proxy(struct setting *s, const char *val)
{
  bool res = settings_default_notify(s, val);

  if (res) {
    lp1_filter_compute_params(&gps_l1ca_config.xcorr_f_params,
                              gps_l1ca_config.xcorr_cof,
                              SECS_MS / GPS_L1CA_BIT_LENGTH_MS);
  }

  return res;
}

/** Register GPS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l1ca_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L1CA_TRACK_SETTING_SECTION,
                             gps_l1ca_config,
                             settings_pov_speed_cof_proxy);
  lp1_filter_compute_params(&gps_l1ca_config.xcorr_f_params,
                            gps_l1ca_config.xcorr_cof,
                            SECS_MS / GPS_L1CA_BIT_LENGTH_MS);

  for (u32 i = 0; i < ARRAY_SIZE(gps_l1ca_trackers); i++) {
    gps_l1ca_trackers[i].active = false;
    gps_l1ca_trackers[i].data = &gps_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l1ca);
}

static void tracker_gps_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l1ca_tracker_data_t));

  tp_tracker_init(channel_info, common_data, &data->data, &gps_l1ca_config);

}

static void tracker_gps_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *data = tracker_data;

  tp_tracker_disable(channel_info, common_data, &data->data);
}

/**
 * Performs ToW caching and propagation.
 *
 * GPS L1 C/A and L2 C use shared structure for ToW caching. When GPS L1 C/A
 * tracker is running, it is responsible for cache updates. Otherwise GPS L2 C
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * GPS L2 C tracker performs ToW update/propagation only on bit edge. This makes
 * it more robust to propagation errors.
 *
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_tow_gps_l1ca(const tracker_channel_info_t *channel_info,
                                tracker_common_data_t *common_data,
                                tp_tracker_data_t *data,
                                u32 cycle_flags)
{
  tp_tow_entry_t tow_entry;

  if (!track_sid_db_load_tow(channel_info->sid, &tow_entry)) {
    /* Error */
    return;
  }

  u64 sample_time_tk = nap_sample_time_to_count(common_data->sample_count);

  bool aligned = false;

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {
    /* Check current state: do we have bit/ToW alignment */
    aligned = true;
  }

  if (TOW_UNKNOWN != common_data->TOW_ms && aligned) {
    /*
     * Verify ToW alignment
     * Current block assumes the bit sync has been reached and current
     * interval has closed a bit interval. ToW shall be aligned by bit
     * duration, which is 20ms for GPS L1 C/A / L2 C.
     */
    u8 tail = common_data->TOW_ms % GPS_L1CA_BIT_LENGTH_MS;
    if (0 != tail) {
      s8 error_ms = tail < (GPS_L1CA_BIT_LENGTH_MS >> 1) ?
                    -tail : GPS_L1CA_BIT_LENGTH_MS - tail;

      log_info_sid(channel_info->sid,
                   "[+%" PRIu32 "ms] Adjusting ToW: "
                   "adjustment=%" PRId8 "ms old_tow=%" PRId32,
                   common_data->update_count,
                   error_ms,
                   common_data->TOW_ms);

      common_data->TOW_ms += error_ms;
    }
  }

  if (TOW_UNKNOWN == common_data->TOW_ms && TOW_UNKNOWN != tow_entry.TOW_ms) {
    /* ToW is not known, but there is a cached value */
    s32 ToW_ms = TOW_UNKNOWN;
    double error_ms = 0;
    u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
    u8 ms_align = aligned ? GPS_L1CA_BIT_LENGTH_MS : GPS_L1CA_PSYMBOL_LENGTH_MS;

    ToW_ms = tp_tow_compute(tow_entry.TOW_ms,
                            time_delta_tk,
                            ms_align,
                            &error_ms);

    if (TOW_UNKNOWN != ToW_ms) {
      log_debug_sid(channel_info->sid,
                    "[+%" PRIu32 "ms] Initializing TOW from cache [%" PRIu8 "ms]"
                    " delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                    common_data->update_count,
                    ms_align,
                    nap_count_to_ms(time_delta_tk),
                    ToW_ms,
                    error_ms);
      common_data->TOW_ms = ToW_ms;
      if (tp_tow_is_sane(common_data->TOW_ms)) {
        common_data->flags |= TRACK_CMN_FLAG_TOW_PROPAGATED;
      } else {
        log_error_sid(channel_info->sid, "[+%"PRIu32"ms] Error TOW propagation %"PRId32,
                      common_data->update_count, common_data->TOW_ms);
        common_data->TOW_ms = TOW_UNKNOWN;
      }
    }
  }

  if (aligned &&
      common_data->cn0 >= CN0_TOW_CACHE_THRESHOLD &&
      data->confirmed) {
    /* Update ToW cache:
     * - bit edge is reached
     * - CN0 is OK
     * - Tracker is confirmed
     */
    tow_entry.TOW_ms = common_data->TOW_ms;
    tow_entry.sample_time_tk = sample_time_tk;
    track_sid_db_update_tow(channel_info->sid, &tow_entry);
  }
}

/**
 * Check L1 doppler vs. other tracked L1 satellite doppler
 *
 * This function whitelists L1 satellite pairs with doppler mismatch.
 * L1 satellites with matching doppler are xcorr flagged for investigation.
 * CN0 difference of xcorr flagged satellites is saved.
 *
 * \param[in]     channel_info    Channel information.
 * \param[in]     common_data     Channel data.
 * \param[in,out] data            Common L1 tracker data.
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be investigated.
 * \param[out]    sat_active      Flags indicating satellites currently tracked.
 * \param[out]    xcorr_cn0_diffs CN0 difference of the satellites to be investigated [dB-Hz].
 *
 * \return None
 */
static void check_L1_entry(const tracker_channel_info_t *channel_info,
                           tracker_common_data_t *common_data,
                           gps_l1ca_tracker_data_t *data,
                           const tracking_channel_cc_entry_t *entry,
                           bool xcorr_flags[],
                           bool sat_active[],
                           float xcorr_cn0_diffs[])
{
  if (CODE_GPS_L1CA != entry->sid.code) {
    /* Ignore other than GPS L1CA for now */
    return;
  }

  if (sid_is_equal(entry->sid, channel_info->sid)) {
    /* Ignore self */
    return;
  }

  /* Mark active SVs. Later clear the whitelist status of inactive SVs */
  sat_active[entry->sid.sat - 1] = true;
  float cn0 = common_data->cn0;
  float freq_mod = fmodf(common_data->xcorr_freq, L1CA_XCORR_FREQ_STEP);
  float entry_cn0 = entry->cn0;
  float entry_freq = entry->freq;
  float entry_freq_mod = fmodf(entry_freq, L1CA_XCORR_FREQ_STEP);
  float error = fabsf(entry_freq_mod - freq_mod);

  s32 max_time_cnt = (s32)(10.0f * gps_l1ca_config.xcorr_time * XCORR_UPDATE_RATE);

  if (error <= gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with matching doppler are xcorr flagged */
    xcorr_flags[entry->sid.sat - 1] = true;
    xcorr_cn0_diffs[entry->sid.sat - 1] = cn0 - entry_cn0;
    data->xcorr_whitelist_counts[entry->sid.sat - 1] = 0;
  } else if (error >= 5.0f * gps_l1ca_config.xcorr_delta) {
    if (data->xcorr_whitelist_counts[entry->sid.sat - 1] < max_time_cnt) {
      ++data->xcorr_whitelist_counts[entry->sid.sat - 1];
    } else {
      /* Signal pair with significant doppler difference is whitelisted
       * after max_time_cnt delay */
      data->xcorr_whitelist[entry->sid.sat - 1] = true;
    }
  }
  if (0 != (entry->flags & TRACKING_CHANNEL_FLAG_FLL_USE)) {
    data->xcorr_whitelist[entry->sid.sat - 1] = false;
  }
}

/**
 * Check xcorr flagged L1 satellites.
 *
 * This function increments the counter when L1 dopplers match.
 * If counter reaches maximum value, then based on cn0 differences
 * xcorr flags are set.
 *
 * \param[in]     channel_info    Channel information.
 * \param[in,out] common_data     Channel data.
 * \param[in,out] data            Common L1 tracker data.
 * \param[in]     idx             Index of the entry.
 * \param[in]     xcorr_flags     Flags indicating satellites to be investigated.
 * \param[in]     xcorr_cn0_diffs CN0 difference of the satellites to be investigated [dB-Hz].
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr suspect.
 *
 * \return None
 */
static void check_L1_xcorr_flags(const tracker_channel_info_t *channel_info,
                                 tracker_common_data_t *common_data,
                                 gps_l1ca_tracker_data_t *data,
                                 u16 idx,
                                 bool xcorr_flags[],
                                 float xcorr_cn0_diffs[],
                                 bool *xcorr_suspect)
{
  if (idx + 1 == channel_info->sid.sat) {
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
        common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
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
 * \param[in]     channel_info    Channel information.
 * \param[in]     common_data     Channel data.
 * \param[in,out] data            Common L1 tracker data.
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flags     Flags indicating satellites to be investigated.
 *
 * \return false if entry was not L2CM from same SV, true if it was.
 */
static bool check_L2_entries(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             gps_l1ca_tracker_data_t *data,
                             const tracking_channel_cc_entry_t *entry,
                             bool *xcorr_flag)
{
  if (CODE_GPS_L2CM != entry->sid.code ||
      entry->sid.sat != channel_info->sid.sat) {
    /* Ignore other than L2CM from same SV */
    return false;
  }

  float freq_mod = fmodf(common_data->xcorr_freq, L1CA_XCORR_FREQ_STEP);
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  /* Convert L2 doppler to L1 */
  float entry_freq = entry->freq * L2_to_L1_freq;
  float entry_freq_mod = fmodf(entry_freq, L1CA_XCORR_FREQ_STEP);
  float error = fabsf(entry_freq_mod - freq_mod);

  if (error <= gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * gps_l1ca_config.xcorr_delta) {
    /* Signal pairs with mismatching doppler are xcorr flagged */
    *xcorr_flag = true;
  }

  if (common_data->cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD) {
    /* Whitelist high CN0 signal */
    data->xcorr_whitelist[channel_info->sid.sat - 1] = true;
  } else {
    data->xcorr_whitelist[channel_info->sid.sat - 1] = false;
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
 * \param[in]     channel_info    Channel information.
 * \param[in,out] common_data     Channel data.
 * \param[in,out] data            Common L1 tracker data.
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr suspect.
 *
 * \return None
 */
static void check_L2_xcorr_flag(const tracker_channel_info_t *channel_info,
                                tracker_common_data_t *common_data,
                                gps_l1ca_tracker_data_t *data,
                                bool xcorr_flag,
                                bool *xcorr_suspect)
{
  s32 max_time_cnt = (s32)(gps_l1ca_config.xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l2 < max_time_cnt) {
      ++data->xcorr_count_l2;
    } else {
      if (data->xcorr_whitelist[channel_info->sid.sat - 1] &&
          data->xcorr_whitelist_l2) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist[channel_info->sid.sat - 1] &&
                 !data->xcorr_whitelist_l2) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist[channel_info->sid.sat - 1]) {
        /* Otherwise if L1 signal is not whitelisted, mark as confirmed xcorr */
        common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
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
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l1_xcorr(const tracker_channel_info_t *channel_info,
                            tracker_common_data_t *common_data,
                            gps_l1ca_tracker_data_t *data,
                            u32 cycle_flags)
{
  if (0 == (cycle_flags & TP_CFLAG_BSYNC_UPDATE) ||
      !tracker_bit_aligned(channel_info->context)) {
    return;
  }

  gps_l1ca_tracker_data_t *mode = data;

  tracking_channel_cc_data_t cc_data;
  u16 cnt = tracking_channel_load_cc_data(&cc_data);

  bool xcorr_flags[NUM_SATS_GPS] = {false};
  bool sat_active[NUM_SATS_GPS] = {false};
  float xcorr_cn0_diffs[NUM_SATS_GPS] = {0.0f};

  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracking_channel_cc_entry_t * const entry = &cc_data.entries[idx];

    check_L1_entry(channel_info, common_data, data, entry,
                   xcorr_flags, sat_active, xcorr_cn0_diffs);
  }

  /* Clear whitelistings of satellites that are no longer tracked */
  for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
    if (!sat_active[idx]) {
      data->xcorr_whitelist[idx] = false;
      data->xcorr_whitelist_counts[idx] = 0;
      data->xcorr_counts[idx] = 0;
    }
  }

  bool sensitivity_mode = tp_tl_is_fll(&mode->data.tl_state);
  /* If signal is in sensitivity mode, all whitelistings are cleared */
  if (sensitivity_mode) {
    for (u16 idx = 0; idx < ARRAY_SIZE(data->xcorr_whitelist); ++idx) {
      data->xcorr_whitelist[idx] = false;
    }
  }

  bool xcorr_suspect = false;
  /* Increment counters or Make decision for all xcorr flagged signals */
  for (u16 idx = 0; idx < ARRAY_SIZE(xcorr_flags); ++idx) {
    check_L1_xcorr_flags(channel_info, common_data, data, idx,
                         xcorr_flags, xcorr_cn0_diffs, &xcorr_suspect);
  }

  bool prn_check_fail = tracker_check_prn_fail_flag(channel_info->context);

  set_xcorr_suspect_flag(channel_info, common_data, data,
                         xcorr_suspect | prn_check_fail, sensitivity_mode);
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
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l1_xcorr_from_l2(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    gps_l1ca_tracker_data_t *data,
                                    u32 cycle_flags)
{
  if (0 == (cycle_flags & TP_CFLAG_BSYNC_UPDATE) ||
      !tracker_bit_aligned(channel_info->context)) {
    return;
  }

  gps_l1ca_tracker_data_t *mode = data;

  tracking_channel_cc_data_t cc_data;
  u16 cnt = tracking_channel_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracking_channel_cc_entry_t * const entry = &cc_data.entries[idx];

    if (check_L2_entries(channel_info, common_data, data,
                         entry, &xcorr_flag)) {
      break;
    }
  }

  bool sensitivity_mode = tp_tl_is_fll(&mode->data.tl_state);
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist[channel_info->sid.sat - 1] = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L2 is xcorr flagged */
  check_L2_xcorr_flag(channel_info, common_data, data,
                      xcorr_flag, &xcorr_suspect);

  bool prn_check_fail = tracker_check_prn_fail_flag(channel_info->context);

  set_xcorr_suspect_flag(channel_info, common_data, data,
                         xcorr_suspect | prn_check_fail, sensitivity_mode);
}

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *l1ca_data = tracker_data;
  tp_tracker_data_t *data = &l1ca_data->data;

  u32 cflags = tp_tracker_update(channel_info, common_data, data,
                                 &gps_l1ca_config);

  /* GPS L1 C/A-specific ToW manipulation */
  update_tow_gps_l1ca(channel_info, common_data, data, cflags);

  /* GPS L1 C/A-specific cross-correlation operations */
  update_l1_xcorr(channel_info, common_data, l1ca_data, cflags);

  /* GPS L1 C/A-specific L2C cross-correlation operations */
  update_l1_xcorr_from_l2(channel_info, common_data, l1ca_data, cflags);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (cflags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {

    /* Start L2 C tracker if not running */
    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_prompt,
                             common_data->carrier_freq,
                             common_data->cn0);
  }
}
