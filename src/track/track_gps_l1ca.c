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

/**
 * GPS L1 C/A tracker data container type.
 */
typedef struct {
  tp_tracker_data_t data;  /**< Tracker data */
} gps_l1ca_tracker_data_t;

/** GPS L1 C/A configuration container */
static tp_tracker_config_t gps_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L1 C/A tracker table */
static tracker_t gps_l1ca_trackers[NUM_GPS_L1CA_TRACKERS]
                                   PLATFORM_TRACK_DATA_TRACKER;
/** GPS L1 C/A tracker data */
static gps_l1ca_tracker_data_t gps_l1ca_tracker_data[NUM_GPS_L1CA_TRACKERS];

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
  .num_trackers = NUM_GPS_L1CA_TRACKERS
};

/** GPS L1 C/A tracker interface list element */
static tracker_interface_list_element_t
tracker_interface_list_element_gps_l1ca = {
  .interface = &tracker_interface_gps_l1ca,
  .next = 0
};

/** Register GPS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l1ca_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L1CA_TRACK_SETTING_SECTION, gps_l1ca_config);

  for (u32 i = 0; i < NUM_GPS_L1CA_TRACKERS; i++) {
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

static void tracker_gps_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l1ca_tracker_data_t *l1ca_data = tracker_data;
  tp_tracker_data_t *data = &l1ca_data->data;

  u32 cflags = tp_tracker_update(channel_info, common_data, data);

  /* GPS L1 C/A-specific ToW manipulation */
  update_tow_gps_l1ca(channel_info, common_data, data, cflags);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (cflags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {

    /* Start L2 C tracker if not running */
    do_l1ca_to_l2cm_handover(common_data->sample_count,
                             channel_info->sid.sat,
                             common_data->code_phase_early,
                             common_data->carrier_freq,
                             common_data->cn0);
  }
}

