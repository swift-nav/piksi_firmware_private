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

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CL_INTERNAL

/* Local headers */
#include "track_gps_l2cl.h"
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
#include <assert.h>

/** GPS L2 C configuration section name */
#define L2CL_TRACK_SETTING_SECTION "l2cl_track"

/** Parameters for carrier phase comparison */
typedef struct {
  double c_L2CL_cp; /**< Current L2CL carrier phase [cycles] */
  double p_L2CL_cp; /**< Previous L2CL carrier phase [cycles] */
  double t_L2CL_cp; /**< L2CL carrier phase to be tested [cycles] */
  double c_L2CM_cp; /**< Current L2CM carrier phase [cycles] */
  double p_L2CM_cp; /**< Previous L2CM carrier phase [cycles] */
  double t_L2CM_cp; /**< L2CM carrier phase to be tested [cycles] */
  s32 c_L2CL_TOW;   /**< Current L2CL TOW [ms] */
  s32 p_L2CL_TOW;   /**< Previous L2CL TOW [ms] */
  s32 c_L2CM_TOW;   /**< Current L2CM TOW [ms] */
  s32 p_L2CM_TOW;   /**< Previous L2CM TOW [ms] */
  u8 count;         /**< Count of matching carrier phase measurements */
} cp_comp_t;

/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2cl_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L2C tracker table */
static tracker_t gps_l2cl_trackers[NUM_GPS_L2CL_TRACKERS];
/** GPS L2C tracker data */
static gps_l2cl_tracker_data_t gps_l2cl_tracker_data[ARRAY_SIZE(gps_l2cl_trackers)];

/* Forward declarations of interface methods for GPS L2C */
static tracker_interface_function_t tracker_gps_l2cl_init;
static tracker_interface_function_t tracker_gps_l2cl_disable;
static tracker_interface_function_t tracker_gps_l2cl_update;

/** GPS L2C tracker interface */
static const tracker_interface_t tracker_interface_gps_l2cl = {
  .code =         CODE_GPS_L2CL,
  .init =         tracker_gps_l2cl_init,
  .disable =      tracker_gps_l2cl_disable,
  .update =       tracker_gps_l2cl_update,
  .trackers =     gps_l2cl_trackers,
  .num_trackers = ARRAY_SIZE(gps_l2cl_trackers)
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cl = {
    .interface = &tracker_interface_gps_l2cl,
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
    lp1_filter_compute_params(&gps_l2cl_config.xcorr_f_params,
                              gps_l2cl_config.xcorr_cof,
                              SECS_MS / GPS_L2C_SYMBOL_LENGTH);
  }

  return res;
}

/** Register L2 CL tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cl_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L2CL_TRACK_SETTING_SECTION,
                             gps_l2cl_config,
                             settings_pov_speed_cof_proxy);
  lp1_filter_compute_params(&gps_l2cl_config.xcorr_f_params,
                            gps_l2cl_config.xcorr_cof,
                            SECS_MS / GPS_L2C_SYMBOL_LENGTH);

  for (u32 i = 0; i < ARRAY_SIZE(gps_l2cl_trackers); i++) {
    gps_l2cl_trackers[i].active = false;
    gps_l2cl_trackers[i].data = &gps_l2cl_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cl);
}

/** Do L2 CM to L2 CL handover.
 *
 * The condition for the handover is the availability of bitsync on L2 CM,
 * and TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void do_l2cm_to_l2cl_handover(u32 sample_count,
                              u16 sat,
                              double code_phase,
                              double carrier_freq,
                              float cn0_init,
                              s32 TOW_ms)
{
  /* compose SID: same SV, but code is L2 CL */
  me_gnss_signal_t mesid = construct_mesid(CODE_GPS_L2CL, sat);

  if (!tracking_startup_ready(mesid)) {
    return; /* L2CL signal from the SV is already in track */
  }

  if ((code_phase < 0) ||
      ((code_phase > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase < (GPS_L2CM_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    log_warn_mesid(mesid,
                   "Unexpected L2CM to L2CL hand-over code phase: %f",
                   code_phase);
    return;
  }

  /* L2CL code starts every 1.5 seconds. Offset must be taken into account. */
  s32 offset_ms = (TOW_ms % GPS_L2CL_PRN_PERIOD);
  u32 code_length = code_to_chip_count(mesid.code);
  u32 chips_in_ms = code_length / GPS_L2CL_PRN_PERIOD;

  if (code_phase > (GPS_L2CM_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)) {
    if (offset_ms == 0) {
      code_phase = GPS_L2CL_CHIPS_NUM - (GPS_L2CM_CHIPS_NUM - code_phase);
    } else {
      code_phase = offset_ms * chips_in_ms - (GPS_L2CM_CHIPS_NUM - code_phase);
    }
  } else {
    code_phase += offset_ms * chips_in_ms;
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .mesid              = mesid,
    .sample_count       = sample_count,
    .carrier_freq       = carrier_freq,
    .code_phase         = code_phase,
    .chips_to_correlate = 1023,
    /* get initial cn0 from parent L2CM channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
  case 0:
    log_debug_mesid(mesid, "L2 CL handover done");
    break;

  case 1:
    /* sat is already in fifo, no need to inform */
    break;

  case 2:
    log_warn_mesid(mesid, "Failed to start L2CL tracking");
    break;

  default:
    assert(!"Unknown code returned");
    break;
  }
}

/**
 * Updates L2CL ToW from cache and propagates it on bit edges.
 *
 * When GPS L1 C/A tracker is running, it is responsible for cache updates.
 * Otherwise GPS L2 CM tracker updates the cache.
 * GPS L2 CL only reads ToW from cache and propagates it on bit edge.
 *
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_tow_gps_l2c(const tracker_channel_info_t *channel_info,
                               tracker_common_data_t *common_data,
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
      u8 tail = common_data->TOW_ms % GPS_L2C_SYMBOL_LENGTH;
      if (0 != tail) {
        s8 error_ms = tail < (GPS_L2C_SYMBOL_LENGTH >> 1) ?
                      -tail : GPS_L2C_SYMBOL_LENGTH - tail;

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
  }
}

static void tracker_gps_l2cl_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  gps_l2cl_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cl_tracker_data_t));

  tp_tracker_init(channel_info, common_data, &data->data, &gps_l2cl_config);

  /* L2CL does not contain data bits.
     L2CL bit sync refers to alignment with L2CM data bits.
     Bit sync is known once we start tracking L2CL, since
     handover from L2CM is done at the end of 20ms integration period,
     i.e. at the edge of a L2CM data bit. */
  tracker_bit_sync_set(channel_info->context, 0);
}

static void tracker_gps_l2cl_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  gps_l2cl_tracker_data_t *data = tracker_data;

  tp_tracker_disable(channel_info, common_data, &data->data);
}

/** Resets cp_sync counter and sets bit polarity to unknown.
 *  This function is called when L2CL data does not match with L2CM data.
 *
 * \param[in,out] common_data Channel data.
 *
 * \return None
 */
static void reset_cp_data(tracker_common_data_t *common_data)
{
  common_data->cp_sync.counter = 0;
  common_data->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
}

/** Load carrier phase and TOW tags for comparison.
 *
 * \param[in]     mesid       ME signal identifier.
 * \param[in,out] cp_comp     Data for carrier phase comparison.
 * \param[in]     common_data Channel data.
 *
 * \return True if L2CM data was found,
 *  and did not have half-cycle ambiguity resolved.
 *  False, otherwise. L2CL will be dropped if False.
 */
static bool load_cp_data(const me_gnss_signal_t mesid, cp_comp_t *cp_comp,
                         tracker_common_data_t *common_data)
{
  bool L2CM_synced = false;

  /* Load L2CL information */
  cp_comp->c_L2CL_cp = common_data->carrier_phase;
  cp_comp->p_L2CL_cp = common_data->carrier_phase_prev;
  cp_comp->c_L2CL_TOW = common_data->TOW_ms;
  cp_comp->p_L2CL_TOW = common_data->TOW_ms_prev;
  cp_comp->count = common_data->cp_sync.counter;

  /* Load L2CM information */
  me_gnss_signal_t mesid_L2CM = construct_mesid(CODE_GPS_L2CM, mesid.sat);
  tracker_channel_t *tracker_channel_L2CM = tracker_channel_get_by_mesid(mesid_L2CM);
  if (tracker_channel_L2CM == NULL) {
    return false;
  }
  tracker_common_data_t *common_data_L2CM = &tracker_channel_L2CM->common_data;

  cp_comp->c_L2CM_cp = common_data_L2CM->carrier_phase;
  cp_comp->p_L2CM_cp = common_data_L2CM->carrier_phase_prev;
  cp_comp->c_L2CM_TOW = common_data_L2CM->TOW_ms;
  cp_comp->p_L2CM_TOW = common_data_L2CM->TOW_ms_prev;
  L2CM_synced = common_data_L2CM->cp_sync.synced;

  /* If L2CM was found and
   * L2CM did not have half-cycle ambiguity resolved, then return true. */
  return (!L2CM_synced);
}

/** Compare carrier phase information and find one with matching TOW.
 *
 * \param[in,out] cp_comp     Data for carrier phase comparison.
 * \param[in,out] common_data Channel data.
 *
 * \return True if matching data was found, False otherwise.
 */
static bool matching_cp_tow(cp_comp_t *cp_comp,
                            tracker_common_data_t *common_data)
{
  /* Find matching TOW and save the carrier phase information. */
  bool TOW_match = true;
  if (cp_comp->c_L2CL_TOW == cp_comp->c_L2CM_TOW) {
    cp_comp->t_L2CL_cp = cp_comp->c_L2CL_cp;
    cp_comp->t_L2CM_cp = cp_comp->c_L2CM_cp;
  } else if (cp_comp->c_L2CL_TOW == cp_comp->p_L2CM_TOW) {
    cp_comp->t_L2CL_cp = cp_comp->c_L2CL_cp;
    cp_comp->t_L2CM_cp = cp_comp->p_L2CM_cp;
  } else if (cp_comp->p_L2CL_TOW == cp_comp->c_L2CM_TOW) {
    cp_comp->t_L2CL_cp = cp_comp->p_L2CL_cp;
    cp_comp->t_L2CM_cp = cp_comp->c_L2CM_cp;
  } else if (cp_comp->p_L2CL_TOW == cp_comp->p_L2CM_TOW) {
    cp_comp->t_L2CL_cp = cp_comp->p_L2CL_cp;
    cp_comp->t_L2CM_cp = cp_comp->p_L2CM_cp;
  } else {
    /* No TOW was matching. */
    TOW_match = false;
  }

  if (!TOW_match) {
    /* If no TOW was matching, reset the counter to zero. */
    reset_cp_data(common_data);
  }
  return TOW_match;
}

/** Compare carrier phase information and find ones with
 *  close to zero, or 0.5 cycle difference.
 *
 * \param[in]     cp_comp  Data for carrier phase comparison.
 * \param[in,out] polarity Polarity of the carrier phase match.
 * \param[in,out] common_data Channel data.
 *
 * \return True if matching data was found, False otherwise.
 */
static bool compare_cp_data(cp_comp_t *cp_comp, s8 *polarity,
                            tracker_common_data_t *common_data)
{
  *polarity = BIT_POLARITY_UNKNOWN;
  bool match = false;
  double test_metric = fabs(remainder(cp_comp->t_L2CL_cp
                                    - cp_comp->t_L2CM_cp, 1.0f));

  /* When the carrier phases match, the test_metric is
   * either close to 0.0, or close to 0.5 [cycles]. */
  if (test_metric < CARRIER_PHASE_TOLERANCE) {
    match = true;
    *polarity = BIT_POLARITY_INVERTED;
  } else if (test_metric > 0.5f - CARRIER_PHASE_TOLERANCE) {
    match = true;
    *polarity = BIT_POLARITY_NORMAL;
  }

  if (!match) {
    /* If carrier phases were not matching, reset the counter to zero. */
    reset_cp_data(common_data);
  }
  return match;
}

/** Increment counter when matching carrier phase has been found.
 *  If counter reaches maximum, declare phase sync, save polarity
 *  and drop L2CL tracker.
 *
 * \param[in]     mesid       ME signal identifier.
 * \param[in]     cp_comp     Data for carrier phase comparison.
 * \param[in]     polarity    Polarity of the carrier phase match.
 * \param[in,out] common_data Channel data.
 *
 * \return None
 */
static void increment_cp_counter(const me_gnss_signal_t mesid, cp_comp_t *cp_comp,
                                 s8 polarity,
                                 tracker_common_data_t *common_data)

{
  if (cp_comp->count < CARRIER_PHASE_AMBIGUITY_COUNTER) {
    /* If counter is below maximum, only increment it. */
    common_data->cp_sync.counter += 1;
  } else {
    /* If counter reached maximum. */
    /* Drop L2CL tracker. */
    common_data->flags |= TRACK_CMN_FLAG_L2CL_AMBIGUITY;
    common_data->cp_sync.synced = true;

    /* Load L2CM information */
    me_gnss_signal_t mesid_L2CM = construct_mesid(CODE_GPS_L2CM, mesid.sat);
    tracker_channel_t *tracker_channel_L2CM = tracker_channel_get_by_mesid(mesid_L2CM);
    if (tracker_channel_L2CM == NULL) {
      return;
    }
    tracker_common_data_t *common_data_L2CM = &tracker_channel_L2CM->common_data;

    /* Update L2CM polarity and sync. */
    common_data_L2CM->cp_sync.polarity = polarity;
    common_data_L2CM->cp_sync.synced = true;
  }
}

/** Main function for comparing carrier phase information
 *  between L2CM and L2CL trackers.
 *
 * \param[in]     mesid       ME signal identifier.
 * \param[in,out] common_data Channel data.
 *
 * \return None
 */
static void process_cp_data(const me_gnss_signal_t mesid,
                            tracker_common_data_t *common_data)
{
  cp_comp_t cp_comp = {0};
  bool data_valid = false;

  /* Check availability of valid L2CM and L2CL data */
  data_valid = load_cp_data(mesid, &cp_comp, common_data);

  /* Drop L2CL tracker if no valid data is available */
  if (!data_valid) {
    tracking_channel_drop_l2cl(mesid);
    return;
  }

  bool TOW_match = true;

  /* Pick phase measurements with matching TOW tag */
  TOW_match =  matching_cp_tow(&cp_comp, common_data);

  /* If no TOW tag matched, skip the round */
  if (!TOW_match) {
    return;
  }

  s8 polarity = BIT_POLARITY_UNKNOWN;
  /* Compare the TOW matched carrier phases */
  bool match = compare_cp_data(&cp_comp, &polarity, common_data);

  /* If carrier phases do not match skip the round */
  if (!match) {
    return;
  }

  /* If carrier phases match, increment counter and
   * declare half-cycle ambiguity resolved when counter
   * reaches maximum value. */
  increment_cp_counter(mesid, &cp_comp, polarity, common_data);
}

static void tracker_gps_l2cl_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cl_tracker_data_t *l2c_data = tracker_data;
  tp_tracker_data_t *data = &l2c_data->data;

  u32 cflags = tp_tracker_update(channel_info, common_data, data,
                                 &gps_l2cl_config);

  /* GPS L2 C-specific ToW manipulation */
  update_tow_gps_l2c(channel_info, common_data, cflags);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (cflags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {
    bool fll_mode = tp_tl_is_fll(&data->tl_state);
    /* Drop L2CL tracker if it is FLL mode */
    if (fll_mode) {
      tracking_channel_drop_l2cl(channel_info->mesid);
    } else if (!common_data->cp_sync.synced) {
      /* Try resolving half-cycle ambiguity if it hasn't been resolved. */
      process_cp_data(channel_info->mesid, common_data);
    }
  }
}
