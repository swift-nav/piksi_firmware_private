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
#include "track.h"
#include "track_cn0.h"
#include "track_gps_l2cl.h" /* for L2CM to L2CL tracking handover */
#include "track_sid_db.h"

/* Non-local headers */
#include <manage.h>
#include <ndb.h>
#include <platform_track.h>
#include <signal.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GPS L2 C configuration section name */
#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2cm_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS L2C */
static tracker_interface_function_t tracker_gps_l2cm_init;
static tracker_interface_function_t tracker_gps_l2cm_update;

/** GPS L2C tracker interface */
static const tracker_interface_t tracker_interface_gps_l2cm = {
    .code = CODE_GPS_L2CM,
    .init = tracker_gps_l2cm_init,
    .disable = tp_tracker_disable,
    .update = tracker_gps_l2cm_update,
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t
    tracker_interface_list_element_gps_l2cm = {
        .interface = &tracker_interface_gps_l2cm, .next = 0};

/**
 * Function for updating configuration on parameter change
 *
 * \param[in] s   Setting descriptor
 * \param[in] val New parameter value
 *
 * \return Update status
 */
static bool settings_pov_speed_cof_proxy(struct setting *s, const char *val) {
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
void track_gps_l2cm_register(void) {
  TP_TRACKER_REGISTER_CONFIG(L2CM_TRACK_SETTING_SECTION,
                             gps_l2cm_config,
                             settings_pov_speed_cof_proxy);
  lp1_filter_compute_params(&gps_l2cm_config.xcorr_f_params,
                            gps_l2cm_config.xcorr_cof,
                            SECS_MS / GPS_L2C_SYMBOL_LENGTH_MS);

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
                              float cn0_init) {
  /* compose L2CM MESID: same SV, but code is L2CM */
  me_gnss_signal_t mesid = construct_mesid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(mesid)) {
    return; /* L2CM signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if (!handover_valid(code_phase, GPS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(
        mesid, "Unexpected L1C/A to L2C handover code phase: %f", code_phase);
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
      .mesid = mesid,
      .sample_count = sample_count,
      /* recalculate doppler freq for L2 from L1 */
      .carrier_freq = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3,
      /* get initial cn0 from parent L1 channel */
      .cn0_init = cn0_init,
      .elevation = TRACKING_ELEVATION_UNKNOWN};

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

static void tracker_gps_l2cm_init(tracker_channel_t *tracker_channel) {
  gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(tracker_channel, &gps_l2cm_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
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
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_tow_gps_l2c(tracker_channel_t *tracker_channel,
                               u32 cycle_flags) {
  tp_tow_entry_t tow_entry;
  me_gnss_signal_t mesid = tracker_channel->mesid;
  gnss_signal_t sid = construct_sid(mesid.code, mesid.sat);
  track_sid_db_load_tow(sid, &tow_entry);

  u64 sample_time_tk = nap_sample_time_to_count(tracker_channel->sample_count);

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(tracker_channel)) {
    if (TOW_UNKNOWN != tracker_channel->TOW_ms) {
      /*
       * Verify ToW alignment
       * Current block assumes the bit sync has been reached and current
       * interval has closed a bit interval. ToW shall be aligned by bit
       * duration, which is 20ms for GPS L1 C/A / L2 C.
       */
      u8 tail = tracker_channel->TOW_ms % GPS_L2C_SYMBOL_LENGTH_MS;
      if (0 != tail) {
        s8 error_ms = tail < (GPS_L2C_SYMBOL_LENGTH_MS >> 1)
                          ? -tail
                          : GPS_L2C_SYMBOL_LENGTH_MS - tail;

        log_error_mesid(mesid,
                        "[+%" PRIu32
                        "ms] TOW error detected: "
                        "error=%" PRId8 "ms old_tow=%" PRId32,
                        tracker_channel->update_count,
                        error_ms,
                        tracker_channel->TOW_ms);

        /* This is rude, but safe. Do not expect it to happen normally. */
        tracker_channel->flags |= TRACKER_FLAG_OUTLIER;
      }
    }

    if (TOW_UNKNOWN == tracker_channel->TOW_ms &&
        TOW_UNKNOWN != tow_entry.TOW_ms) {
      /* ToW is not known, but there is a cached value */
      s32 ToW_ms = TOW_UNKNOWN;
      double error_ms = 0;
      u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
      u8 bit_length = tracker_bit_length_get(tracker_channel);
      ToW_ms = tp_tow_compute(
          tow_entry.TOW_ms, time_delta_tk, bit_length, &error_ms);

      if (TOW_UNKNOWN != ToW_ms) {
        log_debug_mesid(mesid,
                        "[+%" PRIu32
                        "ms]"
                        " Initializing TOW from cache [%" PRIu8
                        "ms] "
                        "delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                        tracker_channel->update_count,
                        bit_length,
                        nap_count_to_ms(time_delta_tk),
                        ToW_ms,
                        error_ms);
        tracker_channel->TOW_ms = ToW_ms;
        if (tp_tow_is_sane(tracker_channel->TOW_ms)) {
          tracker_channel->flags |= TRACKER_FLAG_TOW_VALID;
        } else {
          log_error_mesid(mesid,
                          "[+%" PRIu32 "ms] Error TOW propagation %" PRId32,
                          tracker_channel->update_count,
                          tracker_channel->TOW_ms);
          tracker_channel->TOW_ms = TOW_UNKNOWN;
          tracker_channel->flags &= ~TRACKER_FLAG_TOW_VALID;
        }
      }
    }

    bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
    if ((TOW_UNKNOWN != tracker_channel->TOW_ms) &&
        (tracker_channel->cn0 >= CN0_TOW_CACHE_THRESHOLD) && confirmed &&
        !tracking_is_running(construct_mesid(CODE_GPS_L1CA, mesid.sat))) {
      /* Update ToW cache:
       * - bit edge is reached
       * - CN0 is OK
       * - Tracker is confirmed
       * - There is no GPS L1 C/A tracker for the same SV.
       */
      tow_entry.TOW_ms = tracker_channel->TOW_ms;
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
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     entry           xcorr data to be checked against.
 * \param[out]    xcorr_flag      Flag indicating satellite to be investigated.
 *
 * \return false if entry was not L1C/A from same SV, true if it was.
 */
static bool check_L1_entries(tracker_channel_t *tracker_channel,
                             const tracking_channel_cc_entry_t *entry,
                             bool *xcorr_flag) {
  gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;

  if (CODE_GPS_L1CA != entry->mesid.code ||
      entry->mesid.sat != tracker_channel->mesid.sat) {
    /* Ignore other than L1C/A from same SV */
    return false;
  }

  /* Convert L2 doppler to L1 */
  float L2_to_L1_freq = GPS_L1_HZ / GPS_L2_HZ;
  float freq_mod =
      fmodf(tracker_channel->xcorr_freq * L2_to_L1_freq, L1CA_XCORR_FREQ_STEP);

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

  if (entry_cn0 >= L1CA_XCORR_WHITELIST_THRESHOLD) {
    data->xcorr_whitelist_l1 = true;
  } else {
    data->xcorr_whitelist_l1 = false;
  }

  if (tracker_channel->cn0 >= L2CM_XCORR_WHITELIST_THRESHOLD) {
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
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     xcorr_flag      Flag indicating satellite to be investigated.
 * \param[out]    xcorr_suspect   Flag set if satellite is determined xcorr
 * suspect.
 *
 * \return None
 */
static void check_L1_xcorr_flag(tracker_channel_t *tracker_channel,
                                bool xcorr_flag,
                                bool *xcorr_suspect) {
  gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;
  s32 max_time_cnt = (s32)(gps_l2cm_config.xcorr_time * XCORR_UPDATE_RATE);

  if (xcorr_flag) {
    if (data->xcorr_count_l1 < max_time_cnt) {
      ++data->xcorr_count_l1;
    } else {
      if (data->xcorr_whitelist && data->xcorr_whitelist_l1) {
        /* If both signals are whitelisted, mark as confirmed xcorr */
        tracker_channel->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
      } else if (!data->xcorr_whitelist && !data->xcorr_whitelist_l1) {
        /* If neither signal is whitelisted, mark as xcorr suspect */
        *xcorr_suspect = true;
      } else if (!data->xcorr_whitelist) {
        /* Otherwise if L2 signal is not whitelisted, mark as confirmed xcorr */
        tracker_channel->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
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
 * \param[in,out] tracker_channel Tracker channel data.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l2_xcorr_from_l1(tracker_channel_t *tracker_channel,
                                    u32 cycle_flags) {
  gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;

  if (0 == (cycle_flags & TP_CFLAG_BSYNC_UPDATE) ||
      !tracker_bit_aligned(tracker_channel)) {
    return;
  }

  if (tracker_check_xcorr_flag(tracker_channel)) {
    /* Cross-correlation is set by external thread */
    tracker_channel->flags |= TRACKER_FLAG_XCORR_CONFIRMED;
    return;
  }

  tracking_channel_cc_data_t cc_data;
  u16 cnt = tracking_channel_load_cc_data(&cc_data);

  bool xcorr_flag = false;
  for (u16 idx = 0; idx < cnt; ++idx) {
    const tracking_channel_cc_entry_t *const entry = &cc_data.entries[idx];

    if (check_L1_entries(tracker_channel, entry, &xcorr_flag)) {
      break;
    }
  }

  bool sensitivity_mode =
      (0 != (tracker_channel->flags & TRACKER_FLAG_SENSITIVITY_MODE));
  if (sensitivity_mode) {
    /* If signal is in sensitivity mode, its whitelisting is cleared */
    data->xcorr_whitelist = false;
  }

  bool xcorr_suspect = false;
  /* Increment counter or Make decision if L1 is xcorr flagged */
  check_L1_xcorr_flag(tracker_channel, xcorr_flag, &xcorr_suspect);

  bool prn_check_fail = tracker_check_prn_fail_flag(tracker_channel);

  set_xcorr_suspect_flag(
      tracker_channel, xcorr_suspect | prn_check_fail, sensitivity_mode);
}

/** Read the half-cycle ambiguity status.
 *
 * \param[in,out] tracker_channel Tracker channel data
 *
 * \return Polarity of the data.
 */
static s8 read_data_polarity(tracker_channel_t *tracker_channel) {
  s8 retval = BIT_POLARITY_UNKNOWN;
  /* If the half-cycle ambiguity has been resolved,
   * return polarity, and reset polarity and sync status. */
  if (tracker_channel->cp_sync.synced) {
    retval = tracker_channel->cp_sync.polarity;
    tracker_channel->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    tracker_channel->cp_sync.synced = false;
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
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_l2cl_status(tracker_channel_t *tracker_channel,
                               u32 cycle_flags) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  bool fll_loop = (0 != (tracker_channel->flags & TRACKER_FLAG_FLL_USE));
  bool pll_loop = (0 != (tracker_channel->flags & TRACKER_FLAG_PLL_USE));

  if (fll_loop && !pll_loop) {
    tracker_ambiguity_unknown(tracker_channel);
  } else if (tracker_ambiguity_resolved(tracker_channel)) {
    tracking_channel_drop_l2cl(mesid);
  }

  if ((0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) &&
      (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED)) &&
      (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) &&
      tracker_bit_aligned(tracker_channel)) {
    /* If needed, read half-cycle ambiguity status from L2CL tracker */
    s8 polarity = read_data_polarity(tracker_channel);
    tracker_ambiguity_set(tracker_channel, polarity);

    /* If half-cycle ambiguity is not resolved, try to start L2CL tracker.
     * TOW must be known before trying to start L2CL tracker. */
    if (!tracker_ambiguity_resolved(tracker_channel) &&
        (TOW_UNKNOWN != tracker_channel->TOW_ms)) {
      /* Start L2 CL tracker if not running already */
      do_l2cm_to_l2cl_handover(tracker_channel->sample_count,
                               mesid.sat,
                               tracker_channel->code_phase_prompt,
                               tracker_channel->carrier_freq,
                               tracker_channel->cn0,
                               tracker_channel->TOW_ms);
    }
  }
}

static void tracker_gps_l2cm_update(tracker_channel_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &gps_l2cm_config);

  /* GPS L2 C-specific ToW manipulation */
  update_tow_gps_l2c(tracker_channel, cflags);

  /* GPS L2 C-specific L1 C/A cross-correlation operations */
  update_l2_xcorr_from_l1(tracker_channel, cflags);

  /* GPS L2CL-specific tracking channel operations */
  update_l2cl_status(tracker_channel, cflags);
}
