/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* skip weak attributes for L2CA API implementation */
#define TRACK_GLO_L2CA_INTERNAL

/* Local headers */
#include "track_glo_l2ca.h"
#include "track.h"
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
#include <libswiftnav/glo_map.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GLO L2CA configuration section name */
#define GLO_L2CA_TRACK_SETTING_SECTION "glo_l2ca_track"

static tp_tracker_config_t glo_l2ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GLO L2CA */
static tracker_interface_function_t tracker_glo_l2ca_init;
static tracker_interface_function_t tracker_glo_l2ca_update;

/** GLO L2CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l2ca = {
    .code = CODE_GLO_L2CA,
    .init = tracker_glo_l2ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_glo_l2ca_update,
};

static tracker_interface_list_element_t tracker_interface_list_glo_l2ca = {
    .interface = &tracker_interface_glo_l2ca, .next = 0};

/** Register GLO L2CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l2ca_register(void) {
  TP_TRACKER_REGISTER_CONFIG(
      GLO_L2CA_TRACK_SETTING_SECTION, glo_l2ca_config, settings_default_notify);

  tracker_interface_register(&tracker_interface_list_glo_l2ca);
}

/** Do GLO L1CA to L2CA handover.
 *
 * The condition for the handover is the availability of meander sync on L1CA
 *
 * \param sample_count NAP sample count
 * \param sat GLO L1CA frequency slot FCN
 * \param code_phase_chips L1CA code phase [chips]
 * \param carrier_freq_hz The current Doppler frequency for the L1CA channel
 * \param init_cn0_dbhz CN0 estimate for the L1CA channel [dB-Hz]
 */
void do_glo_l1ca_to_l2ca_handover(u32 sample_count,
                                  u16 sat,
                                  float code_phase_chips,
                                  double carrier_freq_hz,
                                  float init_cn0_dbhz) {
  /* compose L2CA MESID: same SV, but code is L2CA */
  me_gnss_signal_t L2_mesid = construct_mesid(CODE_GLO_L2CA, sat);

  if (!tracking_startup_ready(L2_mesid)) {
    return; /* L2CA signal from the SV is already in track */
  }

  if (!handover_valid(code_phase_chips, GLO_CA_CHIPS_NUM)) {
    log_warn_mesid(L2_mesid,
                   "Unexpected L1CA to L2CA handover code phase: %f",
                   code_phase_chips);
    return;
  }

  /* calculate L2 - L1 frequency scale while taking GLO FCN into account */
  me_gnss_signal_t L1_mesid = construct_mesid(CODE_GLO_L1CA, sat);
  double glo_freq_scale =
      mesid_to_carr_freq(L2_mesid) / mesid_to_carr_freq(L1_mesid);

  u64 extended_sample_count = nap_sample_time_to_count(sample_count);

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
      .mesid = L2_mesid,
      .glo_slot_id = glo_map_get_orbit_slot(sat),
      .sample_count = extended_sample_count,
      /* recalculate doppler freq for L2 from L1 */
      .carrier_freq = carrier_freq_hz * glo_freq_scale,
      .code_phase = code_phase_chips,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(L2_mesid.code) * 1e-3,
      /* get initial cn0 from parent L1 channel */
      .cn0_init = init_cn0_dbhz,
      .elevation = TRACKING_ELEVATION_UNKNOWN};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(L2_mesid, "L2CA handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(L2_mesid, "Failed to start L2CA tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_glo_l2ca_init(tracker_channel_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &glo_l2ca_config);
}

static void tracker_glo_l2ca_update(tracker_channel_t *tracker_channel) {
  u32 tracker_flags = tp_tracker_update(tracker_channel, &glo_l2ca_config);
  (void)tracker_flags;

  /* GLO L2 ToW manipulation */
  update_tow_glo(tracker_channel, tracker_flags);

  /* If GLO SV is marked unhealthy from L2, also drop L1 tracker */
  if (GLO_SV_UNHEALTHY == tracker_channel->health) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_GLO_L1CA, tracker_channel->mesid.sat);
    tracking_channel_drop_unhealthy_glo(mesid_drop);
  }
}
