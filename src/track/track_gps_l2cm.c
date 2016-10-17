/*
 * Copyright (C) 2016 Swift Navigation Inc.
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
#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/**
 * GPS L2C tracker data container type.
 */
typedef struct {
  tp_tracker_data_t data; /**< Common tracker parameters */
} gps_l2cm_tracker_data_t;

/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2cm_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L2C tracker table */
static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
/** GPS L2C tracker data */
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

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
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
    .next = 0
  };

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L2CM_TRACK_SETTING_SECTION, gps_l2cm_config);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
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
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2C signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2C_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
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
    log_debug_sid(sid, "L2 CM handover done");
    break;

  case 1:
    /* sat is already in fifo, no need to inform */
    break;

  case 2:
    log_warn_sid(sid, "Failed to start L2C tracking");
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
  /* Unused parameter */
  (void)tracker_data;

  tp_tracker_disable(channel_info, common_data);
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

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *l2c_data = tracker_data;
  tp_tracker_data_t *data = &l2c_data->data;

  u32 cflags = tp_tracker_update(channel_info, common_data, data);

  /* GPS L2 C-specific ToW manipulation */
  update_tow_gps_l2c(channel_info, common_data, data, cflags);
}
