/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CM_INTERNAL

/* Local headers */
#include "track_gps_l2cm.h"
#include "track_gps_l2cl.h" /* for L2CM to L2CL tracking handover */
#include "track_cn0.h"
#include "track_profile_utils.h"
#include "track_profiles.h"
#include "track_sid_db.h"
#include "track_internal.h"

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
#include <assert.h>

/** GPS L2 C configuration section name */
#define L2CM_TRACK_SETTING_SECTION "l2cm_track"


/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2cm_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L2C tracker table */
static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
/** GPS L2C tracker data */
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[ARRAY_SIZE(gps_l2cm_trackers)];

/* Forward declarations of interface methods for GPS L2C */
static tracker_interface_function_t tracker_gps_l2cm_init;
static tracker_interface_function_t tracker_gps_l2cm_disable;
static tracker_interface_function_t tracker_gps_l2cm_update;

/** GPS L2C tracker interface */
static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = ARRAY_SIZE(gps_l2cm_trackers)
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
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
    lp1_filter_compute_params(&gps_l2cm_config.xcorr_f_params,
                              gps_l2cm_config.xcorr_cof,
                              SECS_MS / GPS_L2C_SYMBOL_LENGTH_MS);
  }

  return res;
}

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L2CM_TRACK_SETTING_SECTION,
                             gps_l2cm_config,
                             settings_pov_speed_cof_proxy);
  lp1_filter_compute_params(&gps_l2cm_config.xcorr_f_params,
                            gps_l2cm_config.xcorr_cof,
                            SECS_MS / GPS_L2C_SYMBOL_LENGTH_MS);

  for (u32 i = 0; i < ARRAY_SIZE(gps_l2cm_trackers); i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param code_phase L1CA code phase [chips]
 * \param carrier_freq The current Doppler frequency for the L1 C/A channel
 * \param cn0 CN0 estimate for the L1 C/A channel
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init)
{
  /* compose SID: same SV, but code is L2 CM */
  me_gnss_signal_t mesid = construct_mesid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(mesid)) {
    return; /* L2C signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase < (GPS_L1CA_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    log_warn_mesid(mesid,
                   "Unexpected L1C/A to L2C handover code phase: %f",
                   code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)) {
    code_phase = GPS_L2CM_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .mesid              = mesid,
    .sample_count       = sample_count,
    /* recalculate doppler freq for L2 from L1*/
    .carrier_freq       = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
    .code_phase         = code_phase,
    .chips_to_correlate = 1023,
    /* get initial cn0 from parent L1 channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
  case 0:
    log_debug_mesid(mesid, "L2 CM handover done");
    break;

  case 1:
    /* sat is already in fifo, no need to inform */
    break;

  case 2:
    log_warn_mesid(mesid, "Failed to start L2C tracking");
    break;

  default:
    assert(!"Unknown code returned");
    break;
  }
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));

  tp_tracker_init(channel_info, common_data, &data->data, &gps_l2cm_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;

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
static void update_tow_gps_l2c(const tracker_channel_info_t *channel_info,
                               tracker_common_data_t *common_data,
                               tp_tracker_data_t *data,
                               u32 cycle_flags)
{
  tp_tow_entry_t tow_entry;
  gnss_signal_t sid = mesid2sid(channel_info->mesid,
                                channel_info->glo_slot_id);
  if (!track_sid_db_load_tow(sid, &tow_entry)) {
    /* Error */
    return;
  }

  u64 sample_time_tk = nap_sample_time_to_count(common_data->sample_count);

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {

    if (TOW_UNKNOWN != common_data->TOW_ms) {
      /*
       * Verify ToW alignment
       * Current block assumes the bit sync has been reached and current
       * interval has closed a bit interval. ToW shall be aligned by bit
       * duration, which is 20ms for GPS L1 C/A / L2 C.
       */
      u8 tail = common_data->TOW_ms % GPS_L2C_SYMBOL_LENGTH_MS;
      if (0 != tail) {
        s8 error_ms = tail < (GPS_L2C_SYMBOL_LENGTH_MS >> 1) ?
                      -tail : GPS_L2C_SYMBOL_LENGTH_MS - tail;

        log_info_mesid(channel_info->mesid,
                       "[+%" PRIu32 "ms] Adjusting ToW:"
                       " adjustment=%" PRId8 "ms old_tow=%" PRId32,
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
      u8 bit_length = tracker_bit_length_get(channel_info->context);
      ToW_ms = tp_tow_compute(tow_entry.TOW_ms,
                              time_delta_tk,
                              bit_length,
                              &error_ms);

      if (TOW_UNKNOWN != ToW_ms) {
        log_debug_mesid(channel_info->mesid,
                        "[+%" PRIu32 "ms]"
                        " Initializing TOW from cache [%" PRIu8 "ms] "
                        "delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                        common_data->update_count,
                        bit_length,
                        nap_count_to_ms(time_delta_tk),
                        ToW_ms,
                        error_ms);
        common_data->TOW_ms = ToW_ms;
        if (tp_tow_is_sane(common_data->TOW_ms)) {
          common_data->flags |= TRACK_CMN_FLAG_TOW_PROPAGATED;
        } else {
          log_error_mesid(channel_info->mesid,
                          "[+%"PRIu32"ms] Error TOW propagation %"PRId32,
                          common_data->update_count, common_data->TOW_ms);
          common_data->TOW_ms = TOW_UNKNOWN;
        }
      }
    }

    if (TOW_UNKNOWN != common_data->TOW_ms &&
        common_data->cn0 >= CN0_TOW_CACHE_THRESHOLD &&
        data->confirmed &&
        !tracking_is_running(construct_mesid(CODE_GPS_L1CA,
                                             channel_info->mesid.sat))) {
      /* Update ToW cache:
       * - bit edge is reached
       * - CN0 is OK
       * - Tracker is confirmed
       * - There is no GPS L1 C/A tracker for the same SV.
       */
      tow_entry.TOW_ms = common_data->TOW_ms;
      tow_entry.sample_time_tk = sample_time_tk;
      track_sid_db_update_tow(sid, &tow_entry);
    }
  }
}

/**
 * Check L2 doppler vs. L1 doppler of the same SV.
 *
 * L2 satellite with mismatching doppler is xcorr flagged for investigation.
 *
 * \param[in]     channel_info    Channel information.
 * \param[in]     common_data     Channel data.
 * \param[in,out] data            Common L2 tracker data.
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flag      Flag indicating satellite to be investigated.
 *
 * \return false if entry was not L1C/A from same SV, true if it was.
 */
static bool check_L1_entries(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             gps_l2cm_tracker_data_t *data,
                             const tracking_channel_cc_entry_t *entry,
                             bool *xcorr_flag)
{
  if (CODE_GPS_L1CA != entry->mesid.code ||
      entry->mesid.sat != channel_info->mesid.sat) {
    /* Ignore other than L1C/A from same SV */
    return false;
  }

  /* Convert L2 doppler to L1 */
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  float freq_mod = fmodf(common_data->xcorr_freq * L2_to_L1_freq,
                         L1CA_XCORR_FREQ_STEP);

  float entry_cn0 = entry->cn0;
  float entry_freq = entry->freq;
  float entry_freq_mod = fmodf(entry_freq, L1CA_XCORR_FREQ_STEP);
  float error = fabsf(entry_freq_mod - freq_mod);

  if (error <= gps_l2cm_config.xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * gps_l2cm_config.xcorr_delta) {
    /* Signal pairs with mismatching doppler are xcorr flagged */
    *xcorr_flag = true;
  }

  if (entry_cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD)
  {
    data->xcorr_whitelist_l1 = true;
  } else {
    data->xcorr_whitelist_l1 = false;
  }

  if (common_data->cn0 >= L2CM_XCORR_WHITELIST_THRESHOLD) {
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
 * \param[in,out] common_data     Channel data.
 * \param[in,out] data            Common L2 tracker data.
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr suspect.
 *
 * \return None
 */
static void check_L1_xcorr_flag(tracker_common_data_t *common_data,
                                gps_l2cm_tracker_data_t *data,
                                bool xcorr_flag,
                                bool *xcorr_suspect)
{
  s32 max_time_cnt = (s32)(gps_l2cm_config.xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l1 < max_time_cnt) {
      ++data->xcorr_count_l1;
    } else {
      if (data->xcorr_whitelist &&
          data->xcorr_whitelist_l1) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist &&
                 !data->xcorr_whitelist_l1) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist) {
        /* Otherwise if L2 signal is not whitelisted, mark as confirmed xcorr */
        common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
      }
    }
  } else {
    /* Reset CC lock counter */
    data->xcorr_count_l1 = 0;
  }
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
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l2_xcorr_from_l1(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    gps_l2cm_tracker_data_t *data,
                                    u32 cycle_flags)
{
  if (0 == (cycle_flags & TP_CFLAG_BSYNC_UPDATE) ||
      !tracker_bit_aligned(channel_info->context)) {
    return;
  }

  if (tracker_check_xcorr_flag(channel_info->context)) {
    /* Cross-correlation is set by external thread */
    common_data->flags |= TRACK_CMN_FLAG_XCORR_CONFIRMED;
    return;
  }

  gps_l2cm_tracker_data_t *mode = data;

  tracking_channel_cc_data_t cc_data;
  u16 cnt = tracking_channel_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracking_channel_cc_entry_t * const entry = &cc_data.entries[idx];

    if (check_L1_entries(channel_info, common_data, data,
                         entry, &xcorr_flag)) {
      break;
    }
  }

  bool sensitivity_mode = tp_tl_is_fll(&mode->data.tl_state);
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L1 is xcorr flagged */
  check_L1_xcorr_flag(common_data, data, xcorr_flag, &xcorr_suspect);

  bool prn_check_fail = tracker_check_prn_fail_flag(channel_info->context);

  set_xcorr_suspect_flag(channel_info, common_data, data,
                         xcorr_suspect | prn_check_fail, sensitivity_mode);
}

/** Read the half-cycle ambiguity status.
 *
 * \param[in,out] common_data Channel data.
 *
 * \return Polarity of the data.
 */
static s8 read_data_polarity(tracker_common_data_t *common_data)
{
  s8 retval = BIT_POLARITY_UNKNOWN;
  /* If the half-cycle ambiguity has been resolved,
   * return polarity, and reset polarity and sync status. */
  if (common_data->cp_sync.synced) {
    retval = common_data->cp_sync.polarity;
    common_data->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    common_data->cp_sync.synced = false;
  }
  return retval;
}

/**
 * Performs the handling of L2CL tracker for half-cycle ambiguity resolution.
 *
 * If L2CM tracker is in FLL mode, the L2CL tracker is dropped and
 * ambiguity is marked as unknown.
 *
 * Otherwise reads half-cycle ambiguity info from L2CL tracker.
 * When ambiguity is resolved, the L2CL tracker is dropped.
 *
 * Also handles the initialization of L2CL tracker when needed.
 *
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l2cl_status(const tracker_channel_info_t *channel_info,
                               tracker_common_data_t *common_data,
                               const tp_tracker_data_t *data,
                               u32 cycle_flags)
{
  if (tp_tl_is_fll(&data->tl_state)) {
    tracking_channel_drop_l2cl(channel_info->mesid);
    tracker_ambiguity_unknown(channel_info->context);
  } else if (data->lock_detect.outp &&
             data->confirmed &&
             0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
             tracker_bit_aligned(channel_info->context)) {

    /* If needed, read half-cycle ambiguity status from L2CL tracker */
    if (tracker_ambiguity_resolved(channel_info->context)) {
      tracking_channel_drop_l2cl(channel_info->mesid);
    } else {
      s8 polarity = read_data_polarity(common_data);
      tracker_ambiguity_set(channel_info->context, polarity);
    }

    /* If half-cycle ambiguity is not resolved, try to start L2CL tracker.
     * TOW must be known before trying to start L2CL tracker. */
    if (!tracker_ambiguity_resolved(channel_info->context) &&
        TOW_UNKNOWN != common_data->TOW_ms) {
      /* Start L2 CL tracker if not running already */
      do_l2cm_to_l2cl_handover(common_data->sample_count,
                               channel_info->mesid.sat,
                               common_data->code_phase_prompt,
                               common_data->carrier_freq,
                               common_data->cn0,
                               common_data->TOW_ms);
    }
  }
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *l2c_data = tracker_data;
  tp_tracker_data_t *data = &l2c_data->data;

  u32 cflags = tp_tracker_update(channel_info, common_data, data,
                                 &gps_l2cm_config);

  /* GPS L2 C-specific ToW manipulation */
  update_tow_gps_l2c(channel_info, common_data, data, cflags);

  /* GPS L2 C-specific L1 C/A cross-correlation operations */
  update_l2_xcorr_from_l1(channel_info, common_data, l2c_data, cflags);

  /* GPS L2CL-specific tracking channel operations */
  update_l2cl_status(channel_info, common_data, data, cflags);
}
