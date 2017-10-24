/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tommi.paakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_gps_l2c.h"
#include "track_cn0.h"
#include "track_sid_db.h"

/* Non-local headers */
#include <manage.h>
#include <ndb.h>
#include <platform_track.h>
#include <signal.h>
#include <track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GPS L2C configuration section name */
#define L2C_TRACK_SETTING_SECTION "l2c_track"
#define NUM_COH_L2C_20MS_SYMB 10

/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2c_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS L2C */
static tracker_interface_function_t tracker_gps_l2c_init;
static tracker_interface_function_t tracker_gps_l2c_update;

/** GPS L2C tracker interface */
static const tracker_interface_t tracker_interface_gps_l2c = {
    .code = CODE_GPS_L2CM,
    .init = tracker_gps_l2c_init,
    .disable = tp_tracker_disable,
    .update = tracker_gps_l2c_update,
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t tracker_interface_list_element_gps_l2c =
    {.interface = &tracker_interface_gps_l2c, .next = 0};

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
    lp1_filter_compute_params(&gps_l2c_config.xcorr_f_params,
                              gps_l2c_config.xcorr_cof,
                              SECS_MS / GPS_L2C_SYMBOL_LENGTH_MS);
  }

  return res;
}

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2c_register(void) {
  TP_TRACKER_REGISTER_CONFIG(
      L2C_TRACK_SETTING_SECTION, gps_l2c_config, settings_pov_speed_cof_proxy);
  lp1_filter_compute_params(&gps_l2c_config.xcorr_f_params,
                            gps_l2c_config.xcorr_cof,
                            SECS_MS / GPS_L2C_SYMBOL_LENGTH_MS);

  tracker_interface_register(&tracker_interface_list_element_gps_l2c);
}

/** Do L1CA to L2C handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void do_l1ca_to_l2c_handover(u32 sample_count,
                             u16 sat,
                             double code_phase,
                             double carrier_freq,
                             float cn0_init,
                             s32 TOW_ms) {
  /* compose L2CM MESID: same SV, but code is L2CM */
  me_gnss_signal_t mesid = construct_mesid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(mesid)) {
    return; /* L2C signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if (!handover_valid(code_phase, GPS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(
        mesid, "Unexpected L1CA to L2C hand-over code phase: %f", code_phase);
    return;
  }

  /* L2CL code starts every 1.5 seconds. Offset must be taken into account. */
  s32 offset_ms = (TOW_ms % GPS_L2CL_PRN_PERIOD_MS);
  u32 code_length = code_to_chip_count(CODE_GPS_L2CL);
  u32 chips_in_ms = code_length / GPS_L2CL_PRN_PERIOD_MS;

  if (code_phase > (GPS_L1CA_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)) {
    if (offset_ms == 0) {
      code_phase = GPS_L2CL_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
    } else {
      code_phase = offset_ms * chips_in_ms - (GPS_L1CA_CHIPS_NUM - code_phase);
    }
  } else {
    code_phase += offset_ms * chips_in_ms;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid,
      .sample_count = sample_count,
      /* recalculate doppler freq for L2 from L1 */
      .carrier_freq = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
      .code_phase = code_phase,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3,
      /* get initial cn0 from parent L1CA channel */
      .cn0_init = cn0_init,
      .elevation = TRACKING_ELEVATION_UNKNOWN};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid, "L2C handover done");
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

static void tracker_gps_l2c_init(tracker_channel_t *tracker_channel) {
  gps_l2cm_tracker_data_t *data = &tracker_channel->gps_l2cm;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(tracker_channel, &gps_l2c_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
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

  if (error <= gps_l2c_config.xcorr_delta) {
    /* Signal pairs with matching doppler are NOT xcorr flagged */
    *xcorr_flag = false;
  } else if (error >= 10.0f * gps_l2c_config.xcorr_delta) {
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
  s32 max_time_cnt = (s32)(gps_l2c_config.xcorr_time * XCORR_UPDATE_RATE);

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

  if (0 == (cycle_flags & TPF_BSYNC_UPD) ||
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

static void tracker_gps_l2c_update(tracker_channel_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &gps_l2c_config);
  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker_channel));

  if (bit_aligned) {
    /* TOW manipulation on bit edge */
    tracker_tow_cache(tracker_channel);
  }

  /* GPS L2C-specific L1 C/A cross-correlation operations */
  update_l2_xcorr_from_l1(tracker_channel, cflags);

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK));

  if (in_phase_lock && confirmed && bit_aligned) {
    /* naturally synched as we track */
    s8 symb_sign = SIGN(tracker_channel->corrs.corr_epl.very_late.I);
    s8 pol_sign = SIGN(tracker_channel->cp_sync.polarity);
    log_debug("G%02d L2C %+2d %+3d",
              tracker_channel->mesid.sat,
              symb_sign,
              tracker_channel->cp_sync.polarity);
    if (symb_sign != pol_sign) {
      tracker_channel->cp_sync.polarity = symb_sign;
      tracker_channel->cp_sync.synced = false;
    } else {
      tracker_channel->cp_sync.polarity += symb_sign;
      if (NUM_COH_L2C_20MS_SYMB == ABS(tracker_channel->cp_sync.polarity)) {
        tracker_channel->cp_sync.synced = true;
        tracker_channel->cp_sync.polarity -= symb_sign; /* saturate */
      } else {
        tracker_channel->cp_sync.synced = false;
      }
    }
    if (tracker_channel->cp_sync.synced) {
      tracker_channel->bit_polarity = ((tracker_channel->cp_sync.polarity) < 0)
                                          ? BIT_POLARITY_NORMAL
                                          : BIT_POLARITY_INVERTED;
    } else {
      tracker_channel->bit_polarity = BIT_POLARITY_UNKNOWN;
    }
    update_bit_polarity_flags(tracker_channel);
  }
}
