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
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CL, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2CL signal from the SV is already in track */
  }

  if ((code_phase < 0) ||
      ((code_phase > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase < (GPS_L2CM_CHIPS_NUM - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    log_warn_sid(sid, "Unexpected L2CM to L2CL handover code phase: %f",
                 code_phase);
    return;
  }

  /* L2CL code starts every 1.5 seconds. Offset must be taken into account. */
  s32 offset_ms = (TOW_ms % GPS_L2CL_PRN_PERIOD);
  u32 code_length = code_to_chip_count(sid.code);
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

  /* Adjust code phase by 1 chip to accommodate zero in the L2CM slot */
  code_phase -= 1.0f;
  if (code_phase < 0.0f) {
    code_phase += GPS_L2CL_CHIPS_NUM;
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    .carrier_freq       = carrier_freq,
    .code_phase         = code_phase,
    /* initial correlation length is 1 chip shorter,
     * since first L2CM zero is skipped */
    .chips_to_correlate = 1022,
    /* get initial cn0 from parent L2CM channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
  case 0:
    log_debug_sid(sid, "L2 CL handover done");
    break;

  case 1:
    /* sat is already in fifo, no need to inform */
    break;

  case 2:
    log_warn_sid(sid, "Failed to start L2CL tracking");
    break;

  default:
    assert(!"Unknown code returned");
    break;
  }
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
  if (!track_sid_db_load_tow(channel_info->sid, &tow_entry)) {
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

        log_info_sid(channel_info->sid,
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
        log_debug_sid(channel_info->sid,
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
          log_error_sid(channel_info->sid, "[+%"PRIu32"ms] Error TOW propagation %"PRId32,
                        common_data->update_count, common_data->TOW_ms);
          common_data->TOW_ms = TOW_UNKNOWN;
        }
      }
    }

    if (TOW_UNKNOWN != common_data->TOW_ms &&
        common_data->cn0 >= CN0_TOW_CACHE_THRESHOLD &&
        data->confirmed &&
        !tracking_is_running(construct_sid(CODE_GPS_L1CA,
                                           channel_info->sid.sat))) {
      /* Update ToW cache:
       * - bit edge is reached
       * - CN0 is OK
       * - Tracker is confirmed
       * - There is no GPS L1 C/A tracker for the same SV.
       */
      tow_entry.TOW_ms = common_data->TOW_ms;
      tow_entry.sample_time_tk = sample_time_tk;
      track_sid_db_update_tow(channel_info->sid, &tow_entry);
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

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);
}

static void tracker_gps_l2cl_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  gps_l2cl_tracker_data_t *data = tracker_data;

  tp_tracker_disable(channel_info, common_data, &data->data);
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
  update_tow_gps_l2c(channel_info, common_data, data, cflags);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (cflags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {
    tracking_channel_cp_sync_update(channel_info->sid,
                                    common_data->carrier_phase,
                                    common_data->TOW_ms);
    bool fll_mode = tp_tl_is_fll(&data->tl_state);
    /* Drop L2CL tracker if it is FLL mode */
    if (fll_mode) {
      tracking_channel_drop_l2cl(channel_info->sid);
    } else {
      tracking_channel_cp_sync_match(channel_info->sid);
    }
  }
  (void) cflags;
}
