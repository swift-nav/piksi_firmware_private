/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_glo_l1ca.h"
#include "track_glo_l2ca.h" /* for L1CA to L2CA tracking handover */
#include "track_cn0.h"
#include "track_profile_utils.h"
#include "track_profiles.h"
#include "track_sid_db.h"
#include "track.h"

/* Non-local headers */
#include <platform_track.h>
#include <signal.h>
#include <manage.h>
#include <ndb.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <string.h>
#include <assert.h>

/** GLO L1CA configuration section name */
#define GLO_L1CA_TRACK_SETTING_SECTION "glo_l1ca_track"

static tp_tracker_config_t glo_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

static tracker_t glo_l1ca_trackers[NUM_GLO_L1CA_TRACKERS];
static glo_l1ca_tracker_data_t glo_l1ca_tracker_data[ARRAY_SIZE(glo_l1ca_trackers)];

/* Forward declarations of interface methods for GLO L1CA */
static tracker_interface_function_t tracker_glo_l1ca_init;
static tracker_interface_function_t tracker_glo_l1ca_disable;
static tracker_interface_function_t tracker_glo_l1ca_update;

/** GLO L1CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l1ca = {
  .code =         CODE_GLO_L1CA,
  .init =         tracker_glo_l1ca_init,
  .disable =      tracker_glo_l1ca_disable,
  .update =       tracker_glo_l1ca_update,
  .trackers =     glo_l1ca_trackers,
  .num_trackers = ARRAY_SIZE(glo_l1ca_trackers)
};

static tracker_interface_list_element_t tracker_interface_list_glo_l1ca = {
  .interface = &tracker_interface_glo_l1ca,
  .next = 0
};

/** Register GLO L1CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l1ca_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(GLO_L1CA_TRACK_SETTING_SECTION,
                             glo_l1ca_config,
                             settings_default_notify);

  for (u32 i = 0; i < ARRAY_SIZE(glo_l1ca_trackers); i++) {
    glo_l1ca_trackers[i].active = false;
    glo_l1ca_trackers[i].data = &glo_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_glo_l1ca);
}

static void tracker_glo_l1ca_init(tracker_channel_t *tracker_channel)
{
  glo_l1ca_tracker_data_t *data = tracker_channel->tracker->data;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(&tracker_channel->info,
                  &tracker_channel->common_data,
                  &data->data,
                  &glo_l1ca_config);
}

static void tracker_glo_l1ca_disable(tracker_channel_t *tracker_channel)
{
  glo_l1ca_tracker_data_t *data = tracker_channel->tracker->data;

  tp_tracker_disable(&tracker_channel->info,
                     &tracker_channel->common_data, &data->data);
}

s32 propagate_tow_from_sid_db(const tracker_channel_info_t *channel_info,
                              tracker_common_data_t *common_data,
                              u64 sample_time_tk,
                              bool half_bit_aligned,
                              s32 *TOW_residual_ns)
{
  assert(channel_info);
  assert(TOW_residual_ns);
  *TOW_residual_ns = 0;

  u16 glo_orbit_slot = tracker_glo_orbit_slot_get(channel_info->context);
  if (!glo_slot_id_is_valid(glo_orbit_slot)) {
    return TOW_UNKNOWN;
  }

  /* GLO slot ID is known */
  gnss_signal_t sid = construct_sid(channel_info->mesid.code, glo_orbit_slot);
  tp_tow_entry_t tow_entry = {
    .TOW_ms = TOW_UNKNOWN,
    .TOW_residual_ns = 0,
    .sample_time_tk = 0
  };

  track_sid_db_load_tow(sid, &tow_entry);
  if (TOW_UNKNOWN == tow_entry.TOW_ms) {
    return TOW_UNKNOWN;
  }

  /* We have a cached GLO TOW */

  double error_ms = 0;
  u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
  u8 half_bit = (GLO_L1CA_BIT_LENGTH_MS / 2);
  u8 ms_align = half_bit_aligned ? half_bit : GLO_PRN_PERIOD_MS;
  s32 TOW_ms;

  TOW_ms = tp_tow_compute(tow_entry.TOW_ms, time_delta_tk, ms_align, &error_ms);
  if (TOW_UNKNOWN == TOW_ms) {
    return TOW_UNKNOWN;
  }

  log_debug_sid(sid,
                "[+%" PRIu32 "ms] Initializing TOW from cache [%" PRIu8 "ms]"
                " delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                common_data->update_count,
                ms_align,
                nap_count_to_ms(time_delta_tk),
                TOW_ms,
                error_ms);

  *TOW_residual_ns = tow_entry.TOW_residual_ns;
  if (tp_tow_is_sane(TOW_ms)) {
    common_data->flags |= TRACK_CMN_FLAG_TOW_PROPAGATED;
  } else {
    log_error_sid(sid, "[+%"PRIu32"ms] Error TOW propagation %"PRId32,
                  common_data->update_count, TOW_ms);
    TOW_ms = TOW_UNKNOWN;
  }

  return TOW_ms;
}

static void update_tow_in_sid_db(const tracker_common_data_t *common_data,
                                 const tracker_channel_info_t *channel_info,
                                 u64 sample_time_tk)
{
  u16 glo_orbit_slot = tracker_glo_orbit_slot_get(channel_info->context);
  if (!glo_slot_id_is_valid(glo_orbit_slot)) {
    return;
  }

  gnss_signal_t sid = construct_sid(channel_info->mesid.code, glo_orbit_slot);

  /* Update ToW cache */
  tp_tow_entry_t tow_entry = {
    .TOW_ms = common_data->TOW_ms,
    .TOW_residual_ns = common_data->TOW_residual_ns,
    .sample_time_tk = sample_time_tk
  };
  track_sid_db_update_tow(sid, &tow_entry);
}

/**
 * Performs ToW caching and propagation.
 *
 * GLO L1 and L2 use shared structure for ToW caching. When GLO L1
 * tracker is running, it is responsible for cache updates. Otherwise GLO L2
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * \param[in]     channel_info   Channel information.
 * \param[in,out] common_data    Channel data with ToW, sample number and other
 *                               runtime values.
 * \param[in]     data           Common tracker data.
 * \param[in]     cycle_flags    Current cycle flags.
 */
static void update_tow_glo_l1ca(const tracker_channel_info_t *channel_info,
                                tracker_common_data_t *common_data,
                                tp_tracker_data_t *data,
                                u32 cycle_flags)
{
  bool half_bit_aligned = false;

  if (0 != (cycle_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {
    half_bit_aligned = true;
  }

  if (TOW_UNKNOWN != common_data->TOW_ms && half_bit_aligned) {
    /*
     * Verify ToW alignment
     * Current block assumes the meander sync has been reached and current
     * interval has closed a meander interval. ToW shall be aligned by meander
     * duration (half bit), which is 10ms for GLO L1CA.
     */
    u8 half_bit = (GLO_L1CA_BIT_LENGTH_MS / 2);
    u8 tail = common_data->TOW_ms % half_bit;
    if (0 != tail) {
      /* If this correction is needed, then there is something wrong
         either with the TOW cache update or with the meander sync */
      s8 error_ms = (tail < half_bit) ? -tail : (GLO_L1CA_BIT_LENGTH_MS - tail);

      log_error_mesid(channel_info->mesid,
                     "[+%" PRIu32 "ms] Adjusting ToW: "
                     "adjustment=%" PRId8 "ms old_tow=%" PRId32,
                     common_data->update_count,
                     error_ms,
                     common_data->TOW_ms);

      common_data->TOW_ms += error_ms;
    }
  }

  u64 sample_time_tk = nap_sample_time_to_count(common_data->sample_count);

  if (TOW_UNKNOWN == common_data->TOW_ms) {
    common_data->TOW_ms = propagate_tow_from_sid_db(channel_info,
                                                 common_data,
                                                 sample_time_tk,
                                                 half_bit_aligned,
                                                 &common_data->TOW_residual_ns);
  }

  if (half_bit_aligned &&
      common_data->cn0 >= CN0_TOW_CACHE_THRESHOLD &&
      data->confirmed) {
    update_tow_in_sid_db(common_data, channel_info, sample_time_tk);
  }
}

static void tracker_glo_l1ca_update(tracker_channel_t *tracker_channel)
{
  glo_l1ca_tracker_data_t *glo_l1ca_data = tracker_channel->tracker->data;
  tp_tracker_data_t *data = &glo_l1ca_data->data;
  u32 tracker_flags = tp_tracker_update(tracker_channel, data, &glo_l1ca_config);

  /* GLO L1 C/A-specific ToW manipulation */
  update_tow_glo_l1ca(&tracker_channel->info, &tracker_channel->common_data, data, tracker_flags);

  /* If GLO SV is marked unhealthy from L1, also drop L2 tracker */
  if (GLO_SV_UNHEALTHY == tracker_channel->common_data.health) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_GLO_L2CA, tracker_channel->info.mesid.sat);
    tracking_channel_drop_unhealthy_glo(mesid_drop);
  }

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (tracker_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(tracker_channel->info.context)) {

    /* Start GLO L2CA tracker if not running */
    do_glo_l1ca_to_l2ca_handover(tracker_channel->common_data.sample_count,
                                 tracker_channel->info.mesid.sat,
                                 tracker_channel->common_data.code_phase_prompt,
                                 tracker_channel->common_data.carrier_freq,
                                 tracker_channel->common_data.cn0);
  }
}
