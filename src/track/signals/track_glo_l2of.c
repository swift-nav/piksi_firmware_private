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

#include "track_glo_l2of.h"

#include <assert.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

#include "acq/manage.h"
#include "platform_track.h"
#include "reacq/search_manager_utils.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"

/** GLO L2CA configuration section name */
#define GLO_L2OF_TRACK_SETTING_SECTION "glo_l2of_track"

/* This value is used for handover process and will be substracted from initial
 *  C/N0 value taken from L1 to eliminate overshoots at the beginning
 *  of L2 tracking
 *  The value was calculated as mean value of L1/L2 signal power ratio */
#define GLO_L2OF_CN0_INIT_ADJUST_DBHZ (3.2f)

static tp_tracker_config_t glo_l2of_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GLO L2CA */
static tracker_interface_function_t tracker_glo_l2of_init;
static tracker_interface_function_t tracker_glo_l2of_update;

/** GLO L2CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l2of = {
    .code = CODE_GLO_L2OF,
    .init = tracker_glo_l2of_init,
    .disable = tp_tracker_disable,
    .update = tracker_glo_l2of_update,
};

/** Register GLO L2CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l2of_register(void) {
  tracker_interface_register(&tracker_interface_glo_l2of);
}

/** Do GLO L1OF to L2OF handover.
 *
 * The condition for the handover is the availability of meander sync on L1CA
 *
 * \param sample_count     NAP sample count
 * \param sat              GLO L1CA frequency slot FCN
 * \param code_phase_chips L1CA code phase [chips]
 * \param doppler_hz       The current Doppler frequency for the L1CA channel
 * \param init_cn0_dbhz    CN0 estimate for the L1CA channel [dB-Hz]
 */
void do_glo_l1of_to_l2of_handover(u32 sample_count,
                                  u16 sat,
                                  float code_phase_chips,
                                  double doppler_hz,
                                  float init_cn0_dbhz) {
  /* compose L2CA MESID: same SV, but code is L2CA */
  me_gnss_signal_t L2_mesid = construct_mesid(CODE_GLO_L2OF, sat);

  if (!tracking_startup_ready(L2_mesid)) {
    return; /* L2CA signal from the SV is already in track */
  }

  if (!handover_valid(code_phase_chips, GLO_CA_CHIPS_NUM)) {
    log_warn_mesid(L2_mesid,
                   "Unexpected L1OF to L2OF handover code phase: %f",
                   code_phase_chips);
    return;
  }

  /* calculate L2 - L1 frequency scale while taking GLO FCN into account */
  me_gnss_signal_t L1_mesid = construct_mesid(CODE_GLO_L1OF, sat);
  double glo_freq_scale =
      mesid_to_carr_freq(L2_mesid) / mesid_to_carr_freq(L1_mesid);

  u64 extended_sample_count = nap_sample_time_to_count(sample_count);

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_init()
     is called. */
  u16 slot = sm_mesid_to_sat(L1_mesid);

  tracking_startup_params_t startup_params = {
      .mesid = L2_mesid,
      .glo_slot_id = slot,
      .sample_count = extended_sample_count,
      /* recalculate doppler freq for L2 from L1 */
      .doppler_hz = (float)(doppler_hz * glo_freq_scale),
      .code_phase = code_phase_chips,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = (u32)lrint(code_to_chip_rate(L2_mesid.code) * 1e-3),
      /* get initial cn0 from parent L1 channel */
      .cn0_init = init_cn0_dbhz - GLO_L2OF_CN0_INIT_ADJUST_DBHZ};

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

static void tracker_glo_l2of_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &glo_l2of_config);

  tracker_bit_sync_set(tracker, /* bit_phase_ref = */ 0);
}

static void tracker_glo_l2of_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &glo_l2of_config);

  /* If GLO SV is marked unhealthy from L2, also drop L1 tracker */
  if (0 != (tracker->flags & TRACKER_FLAG_UNHEALTHY)) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_GLO_L1OF, tracker->mesid.sat);
    tracker_drop_unhealthy(mesid_drop);
    return;
  }

  if (glo_slot_id_is_valid(tracker->glo_orbit_slot)) {
    gnss_signal_t sid = mesid2sid(tracker->mesid, tracker->glo_orbit_slot);
    if (!glo_active(sid)) {
      tracker_drop_unhealthy(tracker->mesid);
      return;
    }
  }

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);
}
